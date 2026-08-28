#include "fruit_picking_arm/serial_protocol.hpp"

#include <algorithm>
#include <array>
#include <climits>
#include <cmath>
#include <stdexcept>

namespace fruit_picking_arm::serial
{

std::uint16_t read_u16(const std::uint8_t * data)
{
  return static_cast<std::uint16_t>(data[0]) |
         static_cast<std::uint16_t>(static_cast<std::uint16_t>(data[1]) << 8);
}

std::uint32_t read_u32(const std::uint8_t * data)
{
  return static_cast<std::uint32_t>(data[0]) |
         (static_cast<std::uint32_t>(data[1]) << 8) |
         (static_cast<std::uint32_t>(data[2]) << 16) |
         (static_cast<std::uint32_t>(data[3]) << 24);
}

std::int32_t read_i32(const std::uint8_t * data) {return static_cast<std::int32_t>(read_u32(data));}
void append_u8(std::vector<std::uint8_t> & out, std::uint8_t v) {out.push_back(v);}
void append_u16(std::vector<std::uint8_t> & out, std::uint16_t v)
{
  out.push_back(static_cast<std::uint8_t>(v));
  out.push_back(static_cast<std::uint8_t>(v >> 8));
}
void append_u32(std::vector<std::uint8_t> & out, std::uint32_t v)
{
  for (int shift = 0; shift < 32; shift += 8) {
    out.push_back(static_cast<std::uint8_t>(v >> shift));
  }
}
void append_i32(std::vector<std::uint8_t> & out, std::int32_t v)
{
  append_u32(out, static_cast<std::uint32_t>(v));
}

std::uint16_t crc16_ccitt(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xffffU;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= static_cast<std::uint16_t>(data[i]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ? static_cast<std::uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::int32_t encode_angle(double radians)
{
  if (!std::isfinite(radians)) {throw std::runtime_error("non-finite joint value");}
  const auto scaled = std::llround(radians * kAngleScale);
  if (scaled < INT32_MIN || scaled > INT32_MAX) {
    throw std::runtime_error("scaled joint value is outside int32 range");
  }
  return static_cast<std::int32_t>(scaled);
}

std::size_t payload_size(MessageType type)
{
  switch (type) {
    case MessageType::kTrajectoryBegin: return 2;
    case MessageType::kTrajectoryPoint: return 54;
    case MessageType::kTrajectoryEnd:
    case MessageType::kStop:
    case MessageType::kHeartbeat: return 0;
    case MessageType::kClawCommand:
    case MessageType::kAck: return 1;
    case MessageType::kRobotState: return 27;
    case MessageType::kMotionDone: return 3;
    case MessageType::kClawResult: return 1;
    case MessageType::kClawState: return 2;
    default: throw std::runtime_error("unknown serial message type");
  }
}

std::vector<std::uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() != payload_size(frame.type)) {
    throw std::runtime_error("invalid fixed payload size");
  }
  std::vector<std::uint8_t> packet;
  packet.reserve(kFrameOverhead + frame.payload.size());
  append_u8(packet, 0xaa); append_u8(packet, 0x55);
  append_u8(packet, static_cast<std::uint8_t>(frame.type));
  append_u16(packet, frame.sequence);
  packet.insert(packet.end(), frame.payload.begin(), frame.payload.end());
  append_u16(packet, crc16_ccitt(packet.data() + 2, packet.size() - 2));
  return packet;
}

std::vector<Frame> FrameParser::feed(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr && size != 0) {throw std::invalid_argument("null serial input");}
  if (size != 0) {buffer_.insert(buffer_.end(), data, data + size);}
  std::vector<Frame> frames;
  static constexpr std::array<std::uint8_t, 2> marker{{0xaa, 0x55}};
  while (buffer_.size() >= kFrameOverhead) {
    const auto found = std::search(buffer_.begin(), buffer_.end(), marker.begin(), marker.end());
    if (found == buffer_.end()) {
      const bool keep_aa = !buffer_.empty() && buffer_.back() == 0xaa;
      const auto count = buffer_.size() - (keep_aa ? 1U : 0U);
      discarded_bytes_ += count;
      buffer_.erase(buffer_.begin(), buffer_.begin() + count);
      break;
    }
    discarded_bytes_ += static_cast<std::size_t>(found - buffer_.begin());
    buffer_.erase(buffer_.begin(), found);
    if (buffer_.size() < kFrameOverhead) {break;}
    std::size_t body_size;
    try {body_size = payload_size(static_cast<MessageType>(buffer_[2]));}
    catch (const std::exception &) {
      ++discarded_bytes_; buffer_.erase(buffer_.begin()); continue;
    }
    const auto total = kFrameOverhead + body_size;
    if (buffer_.size() < total) {break;}
    if (read_u16(buffer_.data() + 5 + body_size) !=
      crc16_ccitt(buffer_.data() + 2, 3 + body_size))
    {
      ++crc_errors_; ++discarded_bytes_; buffer_.erase(buffer_.begin()); continue;
    }
    frames.push_back(Frame{static_cast<MessageType>(buffer_[2]), read_u16(buffer_.data() + 3),
      std::vector<std::uint8_t>(buffer_.begin() + 5, buffer_.begin() + 5 + body_size)});
    buffer_.erase(buffer_.begin(), buffer_.begin() + total);
  }
  return frames;
}

MotionResult decode_motion_result(const std::vector<std::uint8_t> & p)
{
  if (p.size() != 3) {throw std::runtime_error("invalid MOTION_DONE payload");}
  return MotionResult{0, p[0], read_u16(p.data() + 1)};
}

ClawResult decode_claw_result(const std::vector<std::uint8_t> & p)
{
  if (p.size() != 1) {throw std::runtime_error("invalid CLAW_RESULT payload");}
  const auto result = static_cast<ClawResultCode>(p[0]);
  if (result > ClawResultCode::kFault) {
    throw std::runtime_error("invalid CLAW_RESULT value");
  }
  return ClawResult{result};
}

ClawState decode_claw_state(const std::vector<std::uint8_t> & p)
{
  if (p.size() != 2) {throw std::runtime_error("invalid CLAW_STATE payload");}
  const auto state = static_cast<ClawStateCode>(p[0]);
  if (state > ClawStateCode::kFault || (p[1] & ~kClawStateVerifiedFlag) != 0) {
    throw std::runtime_error("invalid CLAW_STATE value or flags");
  }
  return ClawState{state, (p[1] & kClawStateVerifiedFlag) != 0};
}

RobotState decode_robot_state(const std::vector<std::uint8_t> & p)
{
  if (p.size() != 27) {throw std::runtime_error("invalid ROBOT_STATE payload");}
  RobotState state;
  state.mode = p[0]; state.error_code = read_u16(p.data() + 1);
  state.joint_positions.reserve(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    state.joint_positions.push_back(
      static_cast<double>(read_i32(p.data() + 3 + i * 4)) / kAngleScale);
  }
  return state;
}

}  // namespace fruit_picking_arm::serial
