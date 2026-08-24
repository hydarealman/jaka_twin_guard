#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace fruit_picking_arm::serial
{

constexpr std::uint8_t kProtocolVersion = 1;
constexpr std::uint8_t kAckRequired = 1U << 0;
constexpr std::uint8_t kResponse = 1U << 1;
constexpr std::size_t kHeaderSize = 9;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kMaxPayload = 4096;
constexpr double kAngleScale = 1000000.0;

enum class MessageType : std::uint8_t
{
  kHello = 0x01,
  kHeartbeat = 0x02,
  kAck = 0x03,
  kRobotState = 0x04,
  kError = 0x05,
  kFruitTarget = 0x10,
  kTrajectoryBegin = 0x20,
  kTrajectoryPoint = 0x21,
  kTrajectoryEnd = 0x22,
  kAbort = 0x23,
  kGripperCommand = 0x24,
  kMotionResult = 0x30,
};

enum class AckStatus : std::uint8_t
{
  kOk = 0,
  kBadPayload = 1,
  kBusy = 2,
  kUnsupported = 3,
  kCrcError = 4,
  kOutOfRange = 5,
};

enum class ResultCode : std::uint8_t
{
  kSuccess = 0,
  kUnreachable = 1,
  kGraspMissed = 2,
  kObjectDropped = 3,
  kEstop = 4,
  kInvalidTarget = 5,
  kTrajectoryRejected = 6,
  kCancelled = 7,
  kTimeout = 8,
  kInternalError = 9,
};

struct Frame
{
  MessageType type;
  std::uint16_t sequence;
  std::uint8_t flags;
  std::vector<std::uint8_t> payload;
};

struct MotionResult
{
  std::uint16_t command_sequence{0};
  std::uint16_t object_id{0};
  std::uint8_t result_code{0};
  std::uint16_t error_code{0};
};

struct RobotState
{
  std::uint32_t timestamp_ms{0};
  std::uint8_t mode{0};
  std::uint8_t gripper_opening_mm{255};
  std::uint16_t error_code{0};
  std::array<std::int32_t, 3> tcp_xyz_mm{{0, 0, 0}};
  std::array<std::int32_t, 3> tcp_rpy_mdeg{{0, 0, 0}};
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
std::uint32_t crc32(const std::vector<std::vector<std::uint8_t>> & payloads);
std::int32_t encode_angle(double radians);

std::vector<std::uint8_t> encode_frame(const Frame & frame);
MotionResult decode_motion_result(const std::vector<std::uint8_t> & payload);
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
