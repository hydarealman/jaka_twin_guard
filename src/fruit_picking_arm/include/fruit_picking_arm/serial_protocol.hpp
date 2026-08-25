#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace fruit_picking_arm::serial
{

constexpr std::size_t kJointCount = 6;
constexpr std::size_t kMaxTrajectoryPoints = 100;
constexpr std::size_t kMaxPayload = 54;
constexpr std::size_t kFrameOverhead = 7;
constexpr double kAngleScale = 1000000.0;

enum class MessageType : std::uint8_t
{
  kTrajectoryBegin = 0x01,
  kTrajectoryPoint = 0x02,
  kTrajectoryEnd = 0x03,
  kStop = 0x04,
  kHeartbeat = 0x05,
  kClawCommand = 0x06,
  kAck = 0x80,
  kRobotState = 0x81,
  kMotionDone = 0x82,
  kClawResult = 0x83,
};

enum class AckStatus : std::uint8_t
{
  kOk = 0x00,
  kBadData = 0x01,
  kBusy = 0x02,
  kOutOfRange = 0x03,
  kMissingPoint = 0x04,
  kFault = 0x05,
};

enum class ResultCode : std::uint8_t
{
  kSuccess = 0x00,
  kFailed = 0x01,
  kCancelled = 0x02,
  kEstop = 0x03,
  kTimeout = 0x04,
};

enum class ClawAction : std::uint8_t
{
  kOpen = 0x01,
  kClose = 0x02,
  kStop = 0x03,
};

enum class ClawResultCode : std::uint8_t
{
  kCompletedUnverified = 0x00,
  kInterrupted = 0x01,
  kTimeout = 0x02,
  kFault = 0x03,
};

struct Frame
{
  MessageType type;
  std::uint16_t sequence;
  std::vector<std::uint8_t> payload;
};

struct MotionResult
{
  std::uint16_t command_sequence{0};
  std::uint8_t result_code{0};
  std::uint16_t error_code{0};
};

struct ClawResult
{
  ClawResultCode result_code{ClawResultCode::kCompletedUnverified};
};

struct RobotState
{
  std::uint8_t mode{0};
  std::uint16_t error_code{0};
  std::vector<double> joints;
};

struct TrajectoryPoint
{
  std::uint16_t index{0};
  std::uint32_t time_ms{0};
  std::vector<double> positions;
  std::vector<double> velocities;
};

void append_u8(std::vector<std::uint8_t> & output, std::uint8_t value);
void append_u16(std::vector<std::uint8_t> & output, std::uint16_t value);
void append_u32(std::vector<std::uint8_t> & output, std::uint32_t value);
void append_i32(std::vector<std::uint8_t> & output, std::int32_t value);
std::uint16_t read_u16(const std::uint8_t * data);
std::uint32_t read_u32(const std::uint8_t * data);
std::int32_t read_i32(const std::uint8_t * data);
std::uint16_t crc16_ccitt(const std::uint8_t * data, std::size_t size);
std::int32_t encode_angle(double radians);
std::size_t payload_size(MessageType type);
std::vector<std::uint8_t> encode_frame(const Frame & frame);
MotionResult decode_motion_result(const std::vector<std::uint8_t> & payload);
ClawResult decode_claw_result(const std::vector<std::uint8_t> & payload);
RobotState decode_robot_state(const std::vector<std::uint8_t> & payload);

class FrameParser
{
public:
  std::vector<Frame> feed(const std::uint8_t * data, std::size_t size);
  std::size_t crc_errors() const {return crc_errors_;}
  std::size_t discarded_bytes() const {return discarded_bytes_;}

private:
  std::vector<std::uint8_t> buffer_;
  std::size_t crc_errors_{0};
  std::size_t discarded_bytes_{0};
};

}  // namespace fruit_picking_arm::serial
