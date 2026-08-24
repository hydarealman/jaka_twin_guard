#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "jaka_single_arm/serial_protocol.hpp"

namespace
{

std::vector<std::uint8_t> from_hex(const std::string & text)
{
  std::vector<std::uint8_t> result;
  for (std::size_t index = 0; index < text.size(); index += 2) {
    result.push_back(static_cast<std::uint8_t>(std::stoul(text.substr(index, 2), nullptr, 16)));
  }
  return result;
}

TEST(SerialProtocol, UsesCcittFalseKnownVector)
{
  const std::string value = "123456789";
  EXPECT_EQ(
    jaka_single_arm::serial::crc16_ccitt(
      reinterpret_cast<const std::uint8_t *>(value.data()), value.size()),
    0x29b1U);
}

TEST(SerialProtocol, EncodesCrossLanguageTrajectoryPointVector)
{
  const std::vector<std::uint8_t> payload{
    0x01, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    0xa0, 0x86, 0x01, 0x00, 0xc0, 0xf2, 0xfc, 0xff,
    0xe0, 0x93, 0x04, 0x00, 0x80, 0xe5, 0xf9, 0xff,
    0x20, 0xa1, 0x07, 0x00, 0x40, 0xd8, 0xf6, 0xff,
    0x40, 0x42, 0x0f, 0x00, 0xc0, 0xbd, 0xf0, 0xff,
    0x80, 0x84, 0x1e, 0x00, 0x80, 0x7b, 0xe1, 0xff,
    0xc0, 0xc6, 0x2d, 0x00, 0x40, 0x39, 0xd2, 0xff};
  const auto encoded = jaka_single_arm::serial::encode_frame({
    jaka_single_arm::serial::MessageType::kTrajectoryPoint,
    1,
    jaka_single_arm::serial::kAckRequired,
    payload});
  const auto expected = from_hex(
    "aa55012101010038000100000064000000a0860100c0f2fcffe093040080e5f9ff"
    "20a1070040d8f6ff40420f00c0bdf0ff80841e00807be1ffc0c62d004039d2fffd03");
  EXPECT_EQ(encoded, expected);
}

TEST(SerialProtocol, ParserResynchronizesAndReportsCorruption)
{
  const auto packet = jaka_single_arm::serial::encode_frame({
    jaka_single_arm::serial::MessageType::kHeartbeat, 9, 0, {0x01, 0x02}});
  auto corrupt = packet;
  corrupt.back() ^= 0x7f;
  std::vector<std::uint8_t> input{0x10, 0x20};
  input.insert(input.end(), corrupt.begin(), corrupt.end());
  input.insert(input.end(), packet.begin(), packet.end());

  jaka_single_arm::serial::FrameParser parser;
  const auto frames = parser.feed(input.data(), input.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front().sequence, 9U);
  EXPECT_GE(parser.crc_errors(), 1U);
  EXPECT_GE(parser.discarded_bytes(), 2U);
}

TEST(SerialProtocol, DecodesActualRobotStateAngles)
{
  std::vector<std::uint8_t> payload;
  jaka_single_arm::serial::append_u32(payload, 1234);
  jaka_single_arm::serial::append_u8(payload, 1);
  jaka_single_arm::serial::append_u8(payload, 68);
  jaka_single_arm::serial::append_u16(payload, 0);
  for (int value : {500, -20, 400, 180000, 0, 0}) {
    jaka_single_arm::serial::append_i32(payload, value);
  }
  jaka_single_arm::serial::append_u8(payload, 6);
  for (int value : {100000, -200000, 300000, -400000, 500000, -600000}) {
    jaka_single_arm::serial::append_i32(payload, value);
  }

  const auto state = jaka_single_arm::serial::decode_robot_state(payload);
  EXPECT_EQ(state.timestamp_ms, 1234U);
  EXPECT_EQ(state.mode, 1U);
  EXPECT_EQ(state.gripper_opening_mm, 68U);
  ASSERT_EQ(state.joints.size(), 6U);
  EXPECT_DOUBLE_EQ(state.joints[0], 0.1);
  EXPECT_DOUBLE_EQ(state.joints[5], -0.6);
}

}  // namespace
