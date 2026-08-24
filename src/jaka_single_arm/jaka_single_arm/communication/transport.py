"""Threaded serial transport with ACK/retry and message waiting."""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable

from jaka_single_arm.communication.protocol import (
    Ack,
    AckStatus,
    Frame,
    FrameFlags,
    FrameParser,
    MessageType,
    ProtocolError,
    decode_ack,
    encode_ack,
)


class SerialTransportError(RuntimeError):
    pass


class AckRejected(SerialTransportError):
    def __init__(self, ack: Ack):
        super().__init__(f"packet {ack.acked_seq} rejected: {ack.status.name}, error={ack.error_code}")
        self.ack = ack


def open_serial(port: str, baudrate: int, timeout: float = 0.02):
    """Open a pyserial device lazily so protocol tests do not require pyserial."""
    try:
        import serial
    except ImportError as exc:
        raise SerialTransportError(
            "pyserial is required for hardware communication: pip install pyserial"
        ) from exc
    return serial.serial_for_url(port, baudrate=baudrate, timeout=timeout, write_timeout=1.0)


class SerialSession:
    """Owns one serial link and provides reliable request/response primitives."""

    def __init__(
        self,
        serial_device,
        ack_timeout: float = 0.25,
        retries: int = 3,
        read_size: int = 512,
    ):
        self._serial = serial_device
        self._ack_timeout = float(ack_timeout)
        self._retries = int(retries)
        self._read_size = int(read_size)
        self._parser = FrameParser()
        self._sequence = 0
        self._sequence_lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._pending_lock = threading.Lock()
        self._pending: dict[int, tuple[threading.Event, list[Ack]]] = {}
        self._messages: deque[Frame] = deque(maxlen=512)
        self._message_condition = threading.Condition()
        self._callbacks: dict[MessageType, list[Callable[[Frame], None]]] = {}
        self._running = threading.Event()
        self._closed = threading.Event()
        self._reader: threading.Thread | None = None
        self._fatal_error: str | None = None
        self._fatal_lock = threading.Lock()
        self.last_rx_monotonic = 0.0

    @property
    def is_running(self) -> bool:
        return self._running.is_set()

    def start(self) -> None:
        if self._closed.is_set():
            raise SerialTransportError("serial session is closed")
        with self._fatal_lock:
            if self._fatal_error is not None:
                raise SerialTransportError(self._fatal_error)
        if self.is_running:
            return
        self._running.set()
        self._reader = threading.Thread(target=self._read_loop, name="serial-rx", daemon=True)
        self._reader.start()

    def close(self) -> None:
        self._closed.set()
        self._running.clear()
        with self._pending_lock:
            pending = tuple(self._pending.values())
        for event, _holder in pending:
            event.set()
        with self._message_condition:
            self._message_condition.notify_all()
        try:
            self._serial.close()
        finally:
            if self._reader and self._reader is not threading.current_thread():
                self._reader.join(timeout=1.0)

    @property
    def fatal_error(self) -> str | None:
        with self._fatal_lock:
            return self._fatal_error

    def add_callback(self, msg_type: MessageType, callback: Callable[[Frame], None]) -> None:
        self._callbacks.setdefault(msg_type, []).append(callback)

    def next_sequence(self) -> int:
        with self._sequence_lock:
            self._sequence = (self._sequence % 0xFFFF) + 1
            return self._sequence

    def send_message(
        self,
        msg_type: MessageType,
        payload: bytes = b"",
        require_ack: bool = True,
        flags: FrameFlags = FrameFlags.NONE,
    ) -> int:
        seq = self.next_sequence()
        if require_ack:
            flags |= FrameFlags.ACK_REQUIRED
        frame = Frame(msg_type=msg_type, seq=seq, payload=payload, flags=flags)
        self.send_frame(frame, require_ack=require_ack)
        return seq

    def send_frame(self, frame: Frame, require_ack: bool | None = None) -> None:
        if self._closed.is_set():
            raise SerialTransportError("serial session is closed")
        if not self.is_running:
            self.start()
        require_ack = bool(frame.flags & FrameFlags.ACK_REQUIRED) if require_ack is None else require_ack
        packet = frame.encode()
        event = threading.Event()
        holder: list[Ack] = []
        if require_ack:
            with self._pending_lock:
                self._pending[frame.seq] = (event, holder)
        try:
            attempts = self._retries + 1 if require_ack else 1
            for _ in range(attempts):
                with self._write_lock:
                    written = self._serial.write(packet)
                    if written is not None and written != len(packet):
                        raise SerialTransportError(f"partial serial write: {written}/{len(packet)}")
                    flush = getattr(self._serial, "flush", None)
                    if callable(flush):
                        flush()
                if not require_ack:
                    return
                if event.wait(self._ack_timeout):
                    if not holder:
                        raise SerialTransportError(
                            self.fatal_error or "serial session stopped before ACK"
                        )
                    ack = holder[-1]
                    if ack.status != AckStatus.OK:
                        raise AckRejected(ack)
                    return
            raise SerialTransportError(
                f"ACK timeout for {frame.msg_type.name} seq={frame.seq} "
                f"after {attempts} attempts"
            )
        finally:
            if require_ack:
                with self._pending_lock:
                    self._pending.pop(frame.seq, None)

    def wait_for(self, predicate: Callable[[Frame], bool], timeout: float) -> Frame | None:
        deadline = time.monotonic() + timeout
        with self._message_condition:
            while True:
                for frame in tuple(self._messages):
                    if predicate(frame):
                        self._messages.remove(frame)
                        return frame
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._message_condition.wait(remaining)

    def _read_loop(self) -> None:
        while self.is_running:
            try:
                data = self._serial.read(self._read_size)
                if not data:
                    continue
                self.last_rx_monotonic = time.monotonic()
                for frame in self._parser.feed(data):
                    self._dispatch(frame)
            except (OSError, ProtocolError) as exc:
                self._fail(str(exc))
                break
            except Exception as exc:
                self._fail(str(exc))
                break

    def _fail(self, reason: str) -> None:
        if not self.is_running:
            return
        with self._fatal_lock:
            self._fatal_error = f"serial reader stopped: {reason}"
        self._running.clear()
        with self._pending_lock:
            pending = tuple(self._pending.values())
        for event, _holder in pending:
            event.set()
        with self._message_condition:
            self._message_condition.notify_all()

    def _dispatch(self, frame: Frame) -> None:
        if frame.msg_type != MessageType.ACK and frame.flags & FrameFlags.ACK_REQUIRED:
            self._send_inbound_ack(frame.seq)

        if frame.msg_type == MessageType.ACK:
            try:
                ack = decode_ack(frame.payload)
            except (ProtocolError, ValueError):
                return
            with self._pending_lock:
                pending = self._pending.get(ack.acked_seq)
                if pending:
                    event, holder = pending
                    holder.append(ack)
                    event.set()

        with self._message_condition:
            self._messages.append(frame)
            self._message_condition.notify_all()

        for callback in tuple(self._callbacks.get(frame.msg_type, ())):
            try:
                callback(frame)
            except Exception:
                # One UI/ROS callback must not stop the serial reader.
                continue

    def _send_inbound_ack(self, acked_seq: int) -> None:
        try:
            ack = Frame(
                msg_type=MessageType.ACK,
                seq=self.next_sequence(),
                payload=encode_ack(Ack(acked_seq, AckStatus.OK, 0)),
                flags=FrameFlags.RESPONSE,
            ).encode()
            with self._write_lock:
                written = self._serial.write(ack)
                if written is not None and written != len(ack):
                    raise SerialTransportError(
                        f"partial serial ACK write: {written}/{len(ack)}"
                    )
                flush = getattr(self._serial, "flush", None)
                if callable(flush):
                    flush()
        except Exception:
            pass
