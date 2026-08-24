#include "jaka_single_arm/serial_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <climits>
#include <limits>
#include <stdexcept>
#include <utility>

namespace jaka_single_arm::serial
{
namespace
{

constexpr std::uint8_t kKnownFlags = kAckRequired | kResponse;

bool is_known_message_type(std::uint8_t value)
{
  switch (static_cast<MessageType>(value)) {
    case MessageType::kHello:
    case MessageType::kHeartbeat:
    case MessageType::kAck:
    case MessageType::kRobotState:
    case MessageType::kError:
    case MessageType::kFruitTarget:
    case MessageType::kTrajectoryBegin:
    case MessageType::kTrajectoryPoint:
    case MessageType::kTrajectoryEnd:
    case MessageType::kAbort:
    case MessageType::kGripperCommand:
    case MessageType::kMotionResult:
      return true;
    default:
      return false;
  }
}

}  // namespace

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

std::int32_t read_i32(const std::uint8_t * data)
{
  return static_cast<std::int32_t>(read_u32(data));
}

void append_u8(std::vector<std::uint8_t> & output, std::uint8_t value)
{
  output.push_back(value);
}

void append_u16(std::vector<std::uint8_t> & output, std::uint16_t value)
{
  output.push_back(static_cast<std::uint8_t>(value & 0xffU));
  output.push_back(static_cast<std::uint8_t>((value >> 8) & 0xffU));
}

void append_u32(std::vector<std::uint8_t> & output, std::uint32_t value)
{
  for (int shift = 0; shift < 32; shift += 8) {
    output.push_back(static_cast<std::uint8_t>((value >> shift) & 0xffU));
  }
}

void append_i32(std::vector<std::uint8_t> & output, std::int32_t value)
{
  append_u32(output, static_cast<std::uint32_t>(value));
}

std::uint16_t crc16_ccitt(const std::uint8_t * data, std::size_t size)
{
  std::uint16_t crc = 0xffffU;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<std::uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) ?
        static_cast<std::uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::uint32_t crc32(const std::vector<std::vector<std::uint8_t>> & payloads)
{
  std::uint32_t crc = 0xffffffffU;
  for (const auto & payload : payloads) {
    for (const auto byte : payload) {
      crc ^= byte;
      for (int bit = 0; bit < 8; ++bit) {
        crc = (crc & 1U) ? (crc >> 1) ^ 0xedb88320U : crc >> 1;
      }
    }
  }
  return crc ^ 0xffffffffU;
}

std::int32_t encode_angle(double radians)
{
  if (!std::isfinite(radians)) {
    throw std::runtime_error("non-finite joint value");
  }
  const auto scaled = std::llround(radians * kAngleScale);
  if (scaled < INT32_MIN || scaled > INT32_MAX) {
    throw std::runtime_error("scaled joint value is outside int32 range");
  }
  return static_cast<std::int32_t>(scaled);
}

std::vector<std::uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > kMaxPayload) {
    throw std::runtime_error("serial payload exceeds 4096 bytes");
  }
  if (frame.flags & static_cast<std::uint8_t>(~kKnownFlags)) {
    throw std::runtime_error("serial frame contains unknown flags");
  }
  if (!is_known_message_type(static_cast<std::uint8_t>(frame.type))) {
    throw std::runtime_error("serial frame contains unknown message type");
  }
  std::vector<std::uint8_t> packet;
  packet.reserve(kHeaderSize + frame.payload.size() + kCrcSize);
  append_u8(packet, 0xaa);
  append_u8(packet, 0x55);
  append_u8(packet, kProtocolVersion);
  append_u8(packet, static_cast<std::uint8_t>(frame.type));
  append_u8(packet, frame.flags);
  append_u16(packet, frame.sequence);
  append_u16(packet, static_cast<std::uint16_t>(frame.payload.size()));
  packet.insert(packet.end(), frame.payload.begin(), frame.payload.end());
  append_u16(packet, crc16_ccitt(packet.data() + 2, packet.size() - 2));
  return packet;
}

std::vector<Frame> FrameParser::feed(const std::uint8_t * data, std::size_t size)
{
  if (data == nullptr && size != 0) {
    throw std::invalid_argument("null serial input");
  }
  if (size != 0) {
    buffer_.insert(buffer_.end(), data, data + size);
  }
  std::vector<Frame> frames;
  static constexpr std::array<std::uint8_t, 2> marker{{0xaa, 0x55}};
  while (buffer_.size() >= kHeaderSize + kCrcSize) {
    const auto found = std::search(
      buffer_.begin(), buffer_.end(), marker.begin(), marker.end());
    if (found == buffer_.end()) {
      const bool keep_aa = !buffer_.empty() && buffer_.back() == 0xaa;
      const auto discard_count = buffer_.size() - (keep_aa ? 1U : 0U);
      discarded_bytes_ += discard_count;
      buffer_.erase(buffer_.begin(), buffer_.begin() + discard_count);
      break;
    }
    const auto prefix = static_cast<std::size_t>(found - buffer_.begin());
    discarded_bytes_ += prefix;
    buffer_.erase(buffer_.begin(), found);
    if (buffer_.size() < kHeaderSize + kCrcSize) {
      break;
    }
    const auto payload_size = read_u16(buffer_.data() + 7);
    if (payload_size > kMaxPayload) {
      ++discarded_bytes_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    const auto total_size = kHeaderSize + static_cast<std::size_t>(payload_size) + kCrcSize;
    if (buffer_.size() < total_size) {
      break;
    }
    const auto expected_crc = read_u16(buffer_.data() + kHeaderSize + payload_size);
    const auto actual_crc = crc16_ccitt(
      buffer_.data() + 2, kHeaderSize - 2 + payload_size);
    if (buffer_[2] != kProtocolVersion ||
      (buffer_[4] & static_cast<std::uint8_t>(~kKnownFlags)) != 0 ||
      !is_known_message_type(buffer_[3]) || expected_crc != actual_crc)
    {
      ++crc_errors_;
      ++discarded_bytes_;
      buffer_.erase(buffer_.begin());
      continue;
    }
    Frame frame{
      static_cast<MessageType>(buffer_[3]),
      read_u16(buffer_.data() + 5),
      buffer_[4],
      std::vector<std::uint8_t>(
        buffer_.begin() + kHeaderSize,
        buffer_.begin() + kHeaderSize + payload_size)};
    frames.push_back(std::move(frame));
    buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
  }
  return frames;
}

MotionResult decode_motion_result(const std::vector<std::uint8_t> & payload)
{
  if (payload.size() != 7) {
    throw std::runtime_error("invalid MOTION_RESULT payload size");
  }
  return MotionResult{
    read_u16(payload.data()),
    read_u16(payload.data() + 2),
    payload[4],
    read_u16(payload.data() + 5)};
}

RobotState decode_robot_state(const std::vector<std::uint8_t> & payload)
{
  constexpr std::size_t fixed_size = 33;
  if (payload.size() < fixed_size) {
    throw std::runtime_error("truncated ROBOT_STATE payload");
  }
  RobotState state;
  state.timestamp_ms = read_u32(payload.data());
  state.mode = payload[4];
  state.gripper_opening_mm = payload[5];
  state.error_code = read_u16(payload.data() + 6);
  for (std::size_t index = 0; index < 3; ++index) {
    state.tcp_xyz_mm[index] = read_i32(payload.data() + 8 + index * 4);
    state.tcp_rpy_mdeg[index] = read_i32(payload.data() + 20 + index * 4);
  }
  const auto joint_count = payload[32];
  if (joint_count > 16 ||
    payload.size() != fixed_size + static_cast<std::size_t>(joint_count) * 4)
  {
    throw std::runtime_error("invalid ROBOT_STATE joint count");
  }
  state.joints.reserve(joint_count);
  for (std::size_t index = 0; index < joint_count; ++index) {
    state.joints.push_back(
      static_cast<double>(read_i32(payload.data() + fixed_size + index * 4)) /
      kAngleScale);
  }
  return state;
}

}  // namespace jaka_single_arm::serial
