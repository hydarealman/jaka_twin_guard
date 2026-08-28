#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "fruit_picking_arm/serial_protocol.hpp"

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
    fruit_picking_arm::serial::crc16_ccitt(
      reinterpret_cast<const std::uint8_t *>(value.data()), value.size()),
    0x29b1U);
}

TEST(SerialProtocol, EncodesCrossLanguageTrajectoryPointVector)
{
  const std::vector<std::uint8_t> payload{
    0x00, 0x00, 0x64, 0x00, 0x00, 0x00,
    0xa0, 0x86, 0x01, 0x00, 0xc0, 0xf2, 0xfc, 0xff,
    0xe0, 0x93, 0x04, 0x00, 0x80, 0xe5, 0xf9, 0xff,
    0x20, 0xa1, 0x07, 0x00, 0x40, 0xd8, 0xf6, 0xff,
    0x40, 0x42, 0x0f, 0x00, 0xc0, 0xbd, 0xf0, 0xff,
    0x80, 0x84, 0x1e, 0x00, 0x80, 0x7b, 0xe1, 0xff,
    0xc0, 0xc6, 0x2d, 0x00, 0x40, 0x39, 0xd2, 0xff};
  const auto encoded = fruit_picking_arm::serial::encode_frame({
    fruit_picking_arm::serial::MessageType::kTrajectoryPoint,
    1,
    payload});
  const auto expected = from_hex(
    "aa55020100000064000000a0860100c0f2fcffe093040080e5f9ff20a1070040d8"
    "f6ff40420f00c0bdf0ff80841e00807be1ffc0c62d004039d2ff9dc4");
  EXPECT_EQ(encoded, expected);
}

TEST(SerialProtocol, ParserResynchronizesAndReportsCorruption)
{
  const auto packet = fruit_picking_arm::serial::encode_frame({
    fruit_picking_arm::serial::MessageType::kHeartbeat, 9, {}});
  auto corrupt = packet;
  corrupt.back() ^= 0x7f;
  std::vector<std::uint8_t> input{0x10, 0x20};
  input.insert(input.end(), corrupt.begin(), corrupt.end());
  input.insert(input.end(), packet.begin(), packet.end());

  fruit_picking_arm::serial::FrameParser parser;
  const auto frames = parser.feed(input.data(), input.size());
  ASSERT_EQ(frames.size(), 1U);
  EXPECT_EQ(frames.front().sequence, 9U);
  EXPECT_GE(parser.crc_errors(), 1U);
  EXPECT_GE(parser.discarded_bytes(), 2U);
}

TEST(SerialProtocol, DecodesActualRobotStateAngles)
{
  std::vector<std::uint8_t> payload;
  fruit_picking_arm::serial::append_u8(payload, 1);
  fruit_picking_arm::serial::append_u16(payload, 0);
  for (int value : {100000, -200000, 300000, -400000, 500000, -600000}) {
    fruit_picking_arm::serial::append_i32(payload, value);
  }

  const auto state = fruit_picking_arm::serial::decode_robot_state(payload);
  EXPECT_EQ(state.mode, 1U);
  ASSERT_EQ(state.joint_positions.size(), 6U);
  EXPECT_DOUBLE_EQ(state.joint_positions[0], 0.1);
  EXPECT_DOUBLE_EQ(state.joint_positions[5], -0.6);
}

TEST(SerialProtocol, ClawResultContainsOnlyOpenLoopCompletionStatus)
{
  using fruit_picking_arm::serial::ClawResultCode;
  using fruit_picking_arm::serial::Frame;
  using fruit_picking_arm::serial::MessageType;

  const auto result = fruit_picking_arm::serial::decode_claw_result({0x00});
  EXPECT_EQ(result.result_code, ClawResultCode::kCompletedUnverified);

  const auto packet = fruit_picking_arm::serial::encode_frame(
    Frame{MessageType::kClawResult, 0x1234, {0x00}});
  EXPECT_EQ(packet.size(), 8U);
  EXPECT_EQ(packet[3], 0x34U);
  EXPECT_EQ(packet[4], 0x12U);
  EXPECT_EQ(packet[5], 0x00U);
}

TEST(SerialProtocol, DecodesOpenLoopClawStateAndRejectsUnknownFlags)
{
  using fruit_picking_arm::serial::ClawStateCode;

  const auto state = fruit_picking_arm::serial::decode_claw_state({0x02, 0x00});
  EXPECT_EQ(state.state_code, ClawStateCode::kOpen);
  EXPECT_FALSE(state.verified);
  EXPECT_THROW(
    fruit_picking_arm::serial::decode_claw_state({0x02, 0x80}),
    std::runtime_error);
}

}  // namespace
