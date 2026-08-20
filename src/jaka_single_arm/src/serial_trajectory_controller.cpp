#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cerrno>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <poll.h>
#include <set>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <unistd.h>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace jaka_single_arm
{
namespace serial
{

constexpr uint8_t kProtocolVersion = 1;
constexpr uint8_t kAckRequired = 1U << 0;
constexpr uint8_t kResponse = 1U << 1;
constexpr std::size_t kHeaderSize = 9;
constexpr std::size_t kCrcSize = 2;
constexpr std::size_t kMaxPayload = 4096;
constexpr double kAngleScale = 1000000.0;

enum class MessageType : uint8_t
{
  kHeartbeat = 0x02,
  kAck = 0x03,
  kRobotState = 0x04,
  kTrajectoryBegin = 0x20,
  kTrajectoryPoint = 0x21,
  kTrajectoryEnd = 0x22,
  kAbort = 0x23,
  kGripperCommand = 0x24,
  kMotionResult = 0x30,
};

enum class ResultCode : uint8_t
{
  kSuccess = 0,
};

struct Frame
{
  MessageType type;
  uint16_t sequence;
  uint8_t flags;
  std::vector<uint8_t> payload;
};

struct MotionResult
{
  uint16_t command_sequence{0};
  uint16_t object_id{0};
  uint8_t result_code{0};
  uint16_t error_code{0};
};

struct RobotState
{
  uint8_t mode{0};
  uint8_t gripper_opening_mm{255};
  uint16_t error_code{0};
  std::vector<double> joints;
};

struct TrajectoryPoint
{
  uint16_t index;
  uint32_t time_ms;
  std::vector<double> positions;
  std::vector<double> velocities;
};

void append_u8(std::vector<uint8_t> & output, uint8_t value)
{
  output.push_back(value);
}

void append_u16(std::vector<uint8_t> & output, uint16_t value)
{
  output.push_back(static_cast<uint8_t>(value & 0xff));
  output.push_back(static_cast<uint8_t>((value >> 8) & 0xff));
}

void append_u32(std::vector<uint8_t> & output, uint32_t value)
{
  for (int shift = 0; shift < 32; shift += 8) {
    output.push_back(static_cast<uint8_t>((value >> shift) & 0xff));
  }
}

void append_i32(std::vector<uint8_t> & output, int32_t value)
{
  append_u32(output, static_cast<uint32_t>(value));
}

uint16_t read_u16(const uint8_t * data)
{
  return static_cast<uint16_t>(data[0]) |
         static_cast<uint16_t>(data[1] << 8);
}

uint32_t read_u32(const uint8_t * data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

int32_t read_i32(const uint8_t * data)
{
  return static_cast<int32_t>(read_u32(data));
}

uint16_t crc16_ccitt(const uint8_t * data, std::size_t size)
{
  uint16_t crc = 0xffff;
  for (std::size_t index = 0; index < size; ++index) {
    crc ^= static_cast<uint16_t>(data[index]) << 8;
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000) ?
        static_cast<uint16_t>((crc << 1) ^ 0x1021) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

uint32_t crc32(const std::vector<std::vector<uint8_t>> & payloads)
{
  uint32_t crc = 0xffffffffU;
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

int32_t encode_angle(double radians)
{
  if (!std::isfinite(radians)) {
    throw std::runtime_error("non-finite joint value");
  }
  const auto scaled = std::llround(radians * kAngleScale);
  if (scaled < INT32_MIN || scaled > INT32_MAX) {
    throw std::runtime_error("scaled joint value is outside int32 range");
  }
  return static_cast<int32_t>(scaled);
}

std::vector<uint8_t> encode_frame(const Frame & frame)
{
  if (frame.payload.size() > kMaxPayload) {
    throw std::runtime_error("serial payload exceeds 4096 bytes");
  }
  std::vector<uint8_t> packet;
  packet.reserve(kHeaderSize + frame.payload.size() + kCrcSize);
  append_u8(packet, 0xaa);
  append_u8(packet, 0x55);
  append_u8(packet, kProtocolVersion);
  append_u8(packet, static_cast<uint8_t>(frame.type));
  append_u8(packet, frame.flags);
  append_u16(packet, frame.sequence);
  append_u16(packet, static_cast<uint16_t>(frame.payload.size()));
  packet.insert(packet.end(), frame.payload.begin(), frame.payload.end());
  append_u16(packet, crc16_ccitt(packet.data() + 2, packet.size() - 2));
  return packet;
}

class FrameParser
{
public:
  std::vector<Frame> feed(const uint8_t * data, std::size_t size)
  {
    static constexpr std::array<uint8_t, 2> marker{0xaa, 0x55};
    buffer_.insert(buffer_.end(), data, data + size);
    std::vector<Frame> frames;
    while (buffer_.size() >= kHeaderSize + kCrcSize) {
      auto found = std::search(
        buffer_.begin(), buffer_.end(), marker.begin(), marker.end());
      if (found == buffer_.end()) {
        const bool keep_aa = !buffer_.empty() && buffer_.back() == 0xaa;
        buffer_.erase(buffer_.begin(), buffer_.end() - (keep_aa ? 1 : 0));
        break;
      }
      buffer_.erase(buffer_.begin(), found);
      if (buffer_.size() < kHeaderSize + kCrcSize) {
        break;
      }
      const auto payload_size = read_u16(buffer_.data() + 7);
      if (payload_size > kMaxPayload) {
        buffer_.erase(buffer_.begin());
        continue;
      }
      const auto total_size = kHeaderSize + payload_size + kCrcSize;
      if (buffer_.size() < total_size) {
        break;
      }
      const auto expected_crc = read_u16(buffer_.data() + kHeaderSize + payload_size);
      const auto actual_crc = crc16_ccitt(buffer_.data() + 2, kHeaderSize - 2 + payload_size);
      if (buffer_[2] != kProtocolVersion || expected_crc != actual_crc) {
        buffer_.erase(buffer_.begin());
        continue;
      }
      Frame frame{
        static_cast<MessageType>(buffer_[3]),
        read_u16(buffer_.data() + 5),
        buffer_[4],
        std::vector<uint8_t>(
          buffer_.begin() + kHeaderSize,
          buffer_.begin() + kHeaderSize + payload_size)};
      frames.push_back(std::move(frame));
      buffer_.erase(buffer_.begin(), buffer_.begin() + total_size);
    }
    return frames;
  }

private:
  std::vector<uint8_t> buffer_;
};

speed_t baud_constant(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    default:
      throw std::runtime_error("unsupported baudrate: " + std::to_string(baudrate));
  }
}

class PosixSerialPort
{
public:
  PosixSerialPort(const std::string & path, int baudrate)
  {
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error(
              "cannot open serial port " + path + ": " + std::strerror(errno));
    }
    termios options{};
    if (tcgetattr(fd_, &options) != 0) {
      close();
      throw std::runtime_error("tcgetattr failed for " + path);
    }
    cfmakeraw(&options);
    const auto speed = baud_constant(baudrate);
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 1;
    if (tcsetattr(fd_, TCSANOW, &options) != 0) {
      close();
      throw std::runtime_error("tcsetattr failed for " + path);
    }
    tcflush(fd_, TCIOFLUSH);
  }

  ~PosixSerialPort()
  {
    close();
  }

  PosixSerialPort(const PosixSerialPort &) = delete;
  PosixSerialPort & operator=(const PosixSerialPort &) = delete;

  int descriptor() const
  {
    return fd_;
  }

  void write_all(const std::vector<uint8_t> & packet)
  {
    std::size_t offset = 0;
    while (offset < packet.size()) {
      const auto written = ::write(fd_, packet.data() + offset, packet.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd descriptor{fd_, POLLOUT, 0};
        if (::poll(&descriptor, 1, 1000) > 0) {
          continue;
        }
      }
      throw std::runtime_error("serial write failed: " + std::string(std::strerror(errno)));
    }
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_{-1};
};

class SerialLink
{
public:
  using StateCallback = std::function<void(const RobotState &)>;

  SerialLink(
    const std::string & port, int baudrate, double ack_timeout_seconds,
    int retries)
  : port_(port, baudrate),
    ack_timeout_(std::chrono::duration<double>(ack_timeout_seconds)),
    retries_(retries)
  {
    running_.store(true);
    reader_ = std::thread([this]() {read_loop();});
  }

  ~SerialLink()
  {
    close();
  }

  void close()
  {
    running_.store(false);
    port_.close();
    if (reader_.joinable() && reader_.get_id() != std::this_thread::get_id()) {
      reader_.join();
    }
    notify_all_pending();
  }

  bool running() const
  {
    return running_.load();
  }

  std::string fatal_error() const
  {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return fatal_error_;
  }

  void set_state_callback(StateCallback callback)
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    state_callback_ = std::move(callback);
  }

  uint16_t send_message(
    MessageType type, const std::vector<uint8_t> & payload,
    bool require_ack)
  {
    const auto sequence = next_sequence();
    auto pending = std::make_shared<PendingAck>();
    if (require_ack) {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      pending_[sequence] = pending;
    }
    const auto packet = encode_frame(
      Frame{type, sequence, require_ack ? kAckRequired : uint8_t{0}, payload});
    try {
      const auto attempts = require_ack ? retries_ + 1 : 1;
      for (int attempt = 0; attempt < attempts; ++attempt) {
        write_packet(packet);
        if (!require_ack) {
          return sequence;
        }
        std::unique_lock<std::mutex> lock(pending->mutex);
        if (pending->condition.wait_for(
            lock, ack_timeout_, [&]() {return pending->done || !running();}))
        {
          if (!running()) {
            throw std::runtime_error("serial link stopped while waiting for ACK");
          }
          if (pending->status != 0) {
            throw std::runtime_error(
                    "C board rejected packet " + std::to_string(sequence) +
                    ", status=" + std::to_string(pending->status) +
                    ", error=" + std::to_string(pending->error_code));
          }
          erase_pending(sequence);
          return sequence;
        }
      }
      throw std::runtime_error(
              "ACK timeout for sequence " + std::to_string(sequence));
    } catch (...) {
      erase_pending(sequence);
      throw;
    }
  }

  MotionResult send_trajectory(const std::vector<TrajectoryPoint> & points)
  {
    if (points.empty()) {
      throw std::runtime_error("cannot send an empty trajectory");
    }
    const auto trajectory_id = next_object_id(trajectory_id_);
    std::vector<uint8_t> begin;
    append_u16(begin, trajectory_id);
    append_u16(begin, static_cast<uint16_t>(points.size()));
    append_u8(begin, static_cast<uint8_t>(points.front().positions.size()));
    append_u8(begin, 1);
    const auto begin_sequence = send_message(
      MessageType::kTrajectoryBegin, begin, true);

    std::vector<std::vector<uint8_t>> point_payloads;
    try {
      for (const auto & point : points) {
        std::vector<uint8_t> payload;
        append_u16(payload, trajectory_id);
        append_u16(payload, point.index);
        append_u32(payload, point.time_ms);
        for (const auto position : point.positions) {
          append_i32(payload, encode_angle(position));
        }
        for (const auto velocity : point.velocities) {
          append_i32(payload, encode_angle(velocity));
        }
        point_payloads.push_back(payload);
        send_message(MessageType::kTrajectoryPoint, payload, true);
      }
      std::vector<uint8_t> end;
      append_u16(end, trajectory_id);
      append_u16(end, static_cast<uint16_t>(points.size()));
      append_u32(end, crc32(point_payloads));
      send_message(MessageType::kTrajectoryEnd, end, true);
    } catch (...) {
      send_abort_noexcept();
      throw;
    }
    const auto timeout = std::max(
      10.0, static_cast<double>(points.back().time_ms) / 1000.0 + 10.0);
    return wait_result(trajectory_id, begin_sequence, false, timeout);
  }

  MotionResult send_single_motor_gripper(
    uint16_t opening_mm, uint16_t speed_mm_s, uint16_t force_permille)
  {
    const auto command_id = next_object_id(gripper_command_id_);
    std::vector<uint8_t> payload;
    append_u16(payload, command_id);
    append_u8(payload, 3);  // POSITION
    append_u16(payload, opening_mm);
    append_u16(payload, speed_mm_s);
    append_u16(payload, force_permille);
    const auto sequence = send_message(
      MessageType::kGripperCommand, payload, true);
    return wait_result(command_id, sequence, true, 10.0);
  }

  void send_heartbeat()
  {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    const auto timestamp = static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
    std::vector<uint8_t> payload;
    append_u32(payload, timestamp);
    send_message(MessageType::kHeartbeat, payload, false);
  }

  void send_abort_noexcept()
  {
    try {
      send_message(MessageType::kAbort, {}, true);
    } catch (...) {
    }
  }

private:
  struct PendingAck
  {
    std::mutex mutex;
    std::condition_variable condition;
    bool done{false};
    uint8_t status{0};
    uint16_t error_code{0};
  };

  uint16_t next_sequence()
  {
    auto next = ++sequence_;
    if (next == 0) {
      next = ++sequence_;
    }
    return next;
  }

  static uint16_t next_object_id(std::atomic<uint16_t> & value)
  {
    auto next = ++value;
    if (next == 0) {
      next = ++value;
    }
    return next;
  }

  void write_packet(const std::vector<uint8_t> & packet)
  {
    if (!running()) {
      throw std::runtime_error("serial link is not running");
    }
    std::lock_guard<std::mutex> lock(write_mutex_);
    port_.write_all(packet);
  }

  void erase_pending(uint16_t sequence)
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_.erase(sequence);
  }

  void notify_all_pending()
  {
    std::vector<std::shared_ptr<PendingAck>> pending;
    {
      std::lock_guard<std::mutex> lock(pending_mutex_);
      for (const auto & item : pending_) {
        pending.push_back(item.second);
      }
    }
    for (const auto & item : pending) {
      item->condition.notify_all();
    }
    result_condition_.notify_all();
  }

  void read_loop()
  {
    std::array<uint8_t, 512> buffer{};
    try {
      while (running()) {
        pollfd descriptor{port_.descriptor(), POLLIN, 0};
        const auto poll_result = ::poll(&descriptor, 1, 100);
        if (poll_result < 0) {
          if (errno == EINTR) {
            continue;
          }
          throw std::runtime_error("serial poll failed");
        }
        if (poll_result == 0) {
          continue;
        }
        if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) {
          throw std::runtime_error("serial device disconnected");
        }
        const auto count = ::read(
          port_.descriptor(), buffer.data(), buffer.size());
        if (count > 0) {
          for (const auto & frame : parser_.feed(
              buffer.data(), static_cast<std::size_t>(count)))
          {
            dispatch(frame);
          }
        } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK &&
          errno != EINTR)
        {
          throw std::runtime_error(
                  "serial read failed: " + std::string(std::strerror(errno)));
        }
      }
    } catch (const std::exception & error) {
      {
        std::lock_guard<std::mutex> lock(error_mutex_);
        fatal_error_ = error.what();
      }
      running_.store(false);
      notify_all_pending();
    }
  }

  void dispatch(const Frame & frame)
  {
    if (frame.type != MessageType::kAck && (frame.flags & kAckRequired)) {
      std::vector<uint8_t> ack;
      append_u16(ack, frame.sequence);
      append_u8(ack, 0);
      append_u16(ack, 0);
      const auto packet = encode_frame(
        Frame{MessageType::kAck, next_sequence(), kResponse, ack});
      write_packet(packet);
    }
    if (frame.type == MessageType::kAck) {
      if (frame.payload.size() != 5) {
        return;
      }
      const auto acked_sequence = read_u16(frame.payload.data());
      std::shared_ptr<PendingAck> pending;
      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto found = pending_.find(acked_sequence);
        if (found != pending_.end()) {
          pending = found->second;
        }
      }
      if (pending) {
        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          pending->status = frame.payload[2];
          pending->error_code = read_u16(frame.payload.data() + 3);
          pending->done = true;
        }
        pending->condition.notify_all();
      }
      return;
    }
    if (frame.type == MessageType::kMotionResult) {
      if (frame.payload.size() != 7) {
        return;
      }
      MotionResult result{
        read_u16(frame.payload.data()),
        read_u16(frame.payload.data() + 2),
        frame.payload[4],
        read_u16(frame.payload.data() + 5)};
      {
        std::lock_guard<std::mutex> lock(result_mutex_);
        results_.push_back(result);
        if (results_.size() > 128) {
          results_.pop_front();
        }
      }
      result_condition_.notify_all();
      return;
    }
    if (frame.type == MessageType::kRobotState) {
      const auto state = decode_robot_state(frame.payload);
      StateCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = state_callback_;
      }
      if (callback) {
        callback(state);
      }
    }
  }

  static RobotState decode_robot_state(const std::vector<uint8_t> & payload)
  {
    constexpr std::size_t fixed_size = 33;
    if (payload.size() < fixed_size) {
      throw std::runtime_error("truncated ROBOT_STATE payload");
    }
    RobotState state;
    state.mode = payload[4];
    state.gripper_opening_mm = payload[5];
    state.error_code = read_u16(payload.data() + 6);
    const auto joint_count = payload[32];
    if (payload.size() != fixed_size + static_cast<std::size_t>(joint_count) * 4) {
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

  MotionResult wait_result(
    uint16_t object_id, uint16_t command_sequence, bool strict_sequence,
    double timeout_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    std::unique_lock<std::mutex> lock(result_mutex_);
    while (running()) {
      const auto found = std::find_if(
        results_.begin(), results_.end(),
        [&](const MotionResult & result) {
          return result.object_id == object_id &&
                 (!strict_sequence || result.command_sequence == command_sequence);
        });
      if (found != results_.end()) {
        const auto result = *found;
        results_.erase(found);
        return result;
      }
      if (result_condition_.wait_until(lock, deadline) == std::cv_status::timeout) {
        break;
      }
    }
    throw std::runtime_error(
            "motion result timeout for object " + std::to_string(object_id));
  }

  PosixSerialPort port_;
  std::chrono::duration<double> ack_timeout_;
  int retries_;
  std::atomic<bool> running_{false};
  std::atomic<uint16_t> sequence_{0};
  std::atomic<uint16_t> trajectory_id_{0};
  std::atomic<uint16_t> gripper_command_id_{0};
  std::thread reader_;
  FrameParser parser_;
  std::mutex write_mutex_;
  std::mutex pending_mutex_;
  std::map<uint16_t, std::shared_ptr<PendingAck>> pending_;
  std::mutex result_mutex_;
  std::condition_variable result_condition_;
  std::deque<MotionResult> results_;
  std::mutex callback_mutex_;
  StateCallback state_callback_;
  mutable std::mutex error_mutex_;
  std::string fatal_error_;
};

}  // namespace serial

class SerialTrajectoryController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  SerialTrajectoryController()
  : Node("serial_trajectory_controller")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    ack_timeout_ = declare_parameter<double>("ack_timeout", 0.25);
    retries_ = declare_parameter<int>("retries", 3);
    action_name_ = declare_parameter<std::string>(
      "action_name", "/arm_controller/follow_joint_trajectory");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
    require_ready_ = declare_parameter<bool>("require_ready", true);
    robot_state_timeout_s_ = declare_parameter<double>("robot_state_timeout_s", 0.5);
    heartbeat_rate_ = declare_parameter<double>("heartbeat_rate", 2.0);
    arm_joints_ = declare_parameter<std::vector<std::string>>(
      "arm_joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    gripper_joints_ = declare_parameter<std::vector<std::string>>(
      "gripper_joint_names", {"left_finger_joint", "right_finger_joint"});
    lower_limits_ = declare_parameter<std::vector<double>>(
      "arm_lower_limits",
      {-2.792526803, -0.959931089, -1.483529864,
        -2.879793266, -1.483529864, -2.705260341});
    upper_limits_ = declare_parameter<std::vector<double>>(
      "arm_upper_limits",
      {2.792526803, 1.308996939, 1.483529864,
        2.879793266, 1.483529864, 2.705260341});
    gripper_lower_limits_ = declare_parameter<std::vector<double>>(
      "gripper_lower_limits", {0.0, -0.056});
    gripper_upper_limits_ = declare_parameter<std::vector<double>>(
      "gripper_upper_limits", {0.056, 0.0});
    finger_thickness_m_ = declare_parameter<double>(
      "gripper_finger_thickness_m", 0.012);
    gripper_coupling_tolerance_m_ = declare_parameter<double>(
      "gripper_coupling_tolerance_m", 0.0005);
    max_opening_mm_ = declare_parameter<int>("gripper_max_opening_mm", 100);
    gripper_motor_speed_mm_s_ = declare_parameter<int>(
      "gripper_motor_speed_mm_s", 100);
    gripper_motor_force_permille_ = declare_parameter<int>(
      "gripper_motor_force_permille", 500);

    if (arm_joints_.size() != 6 ||
      arm_joints_.size() != lower_limits_.size() ||
      arm_joints_.size() != upper_limits_.size())
    {
      throw std::runtime_error("exactly six arm joints and six arm limits are required");
    }
    if (gripper_joints_.size() != 2 ||
      gripper_lower_limits_.size() != 2 ||
      gripper_upper_limits_.size() != 2)
    {
      throw std::runtime_error("exactly two gripper joints and limits are required");
    }
    if (robot_state_timeout_s_ <= 0.0 || gripper_coupling_tolerance_m_ < 0.0 ||
      max_opening_mm_ < 1 || max_opening_mm_ > 100 ||
      gripper_motor_speed_mm_s_ < 0 || gripper_motor_speed_mm_s_ > 65535 ||
      gripper_motor_force_permille_ < 0 || gripper_motor_force_permille_ > 1000)
    {
      throw std::runtime_error("invalid gripper serial parameters");
    }

    link_ = std::make_unique<serial::SerialLink>(
      serial_port_, baudrate_, ack_timeout_, retries_);
    link_->set_state_callback(
      [this](const serial::RobotState & state) {on_robot_state(state);});
    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_, 20);
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this, action_name_,
      std::bind(
        &SerialTrajectoryController::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &SerialTrajectoryController::handle_cancel, this,
        std::placeholders::_1),
      std::bind(
        &SerialTrajectoryController::handle_accepted, this,
        std::placeholders::_1));
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(0.1, heartbeat_rate_));
    heartbeat_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&SerialTrajectoryController::heartbeat, this));
    RCLCPP_INFO(
      get_logger(),
      "C++ serial controller ready: %s @ %d, 6 arm motors + 1 coupled-jaw gripper motor",
      serial_port_.c_str(), baudrate_);
  }

  ~SerialTrajectoryController() override
  {
    if (link_) {
      link_->close();
    }
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    const auto & trajectory = goal->trajectory;
    if (trajectory.joint_names.empty() || trajectory.points.empty()) {
      RCLCPP_WARN(get_logger(), "Rejecting empty trajectory");
      return rclcpp_action::GoalResponse::REJECT;
    }
    const std::set<std::string> requested(
      trajectory.joint_names.begin(), trajectory.joint_names.end());
    if (trajectory.points.size() > 65535) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory with too many points");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (requested.size() != trajectory.joint_names.size()) {
      RCLCPP_WARN(get_logger(), "Rejecting duplicate joint names");
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto & name : arm_joints_) {
      if (requested.count(name) == 0) {
        RCLCPP_WARN(get_logger(), "Rejecting trajectory missing %s", name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    const bool has_left = requested.count(gripper_joints_[0]) != 0;
    const bool has_right = requested.count(gripper_joints_[1]) != 0;
    if (has_left != has_right) {
      RCLCPP_WARN(get_logger(), "Rejecting partial gripper command");
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto & name : requested) {
      if (std::find(arm_joints_.begin(), arm_joints_.end(), name) == arm_joints_.end() &&
        std::find(gripper_joints_.begin(), gripper_joints_.end(), name) ==
        gripper_joints_.end())
      {
        RCLCPP_WARN(get_logger(), "Rejecting unknown joint %s", name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    std::unordered_map<std::string, std::size_t> indices;
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      indices[trajectory.joint_names[index]] = index;
    }
    int64_t previous_ns = -1;
    for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      const auto & point = trajectory.points[point_index];
      if (point.positions.size() != trajectory.joint_names.size() ||
        (!point.velocities.empty() &&
        point.velocities.size() != trajectory.joint_names.size()))
      {
        RCLCPP_WARN(get_logger(), "Rejecting malformed trajectory point");
        return rclcpp_action::GoalResponse::REJECT;
      }
      const auto time_ns =
        static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
        point.time_from_start.nanosec;
      if (time_ns <= previous_ns) {
        RCLCPP_WARN(get_logger(), "Rejecting non-increasing trajectory time");
        return rclcpp_action::GoalResponse::REJECT;
      }
      previous_ns = time_ns;
      for (std::size_t joint = 0; joint < arm_joints_.size(); ++joint) {
        const auto value = point.positions[indices.at(arm_joints_[joint])];
        if (!std::isfinite(value) ||
          value < lower_limits_[joint] || value > upper_limits_[joint])
        {
          RCLCPP_ERROR(
            get_logger(), "Rejecting unsafe %s=%.6f at point %zu",
            arm_joints_[joint].c_str(), value, point_index);
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
      if (has_left) {
        for (std::size_t joint = 0; joint < gripper_joints_.size(); ++joint) {
          const auto value = point.positions[indices.at(gripper_joints_[joint])];
          if (!std::isfinite(value) ||
            value < gripper_lower_limits_[joint] ||
            value > gripper_upper_limits_[joint])
          {
            RCLCPP_ERROR(
              get_logger(), "Rejecting unsafe %s=%.6f",
              gripper_joints_[joint].c_str(), value);
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
        const auto left = point.positions[indices.at(gripper_joints_[0])];
        const auto right = point.positions[indices.at(gripper_joints_[1])];
        if (std::abs(left + right) > gripper_coupling_tolerance_m_) {
          RCLCPP_ERROR(
            get_logger(),
            "Rejecting asymmetric jaw targets at point %zu: left=%.6f right=%.6f; "
            "one physical gripper motor requires coupled jaws",
            point_index, left, right);
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (active_goal_) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    if (require_ready_) {
      const auto last_state_ns = last_state_steady_ns_.load();
      const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto state_age_s = last_state_ns == 0 ?
        std::numeric_limits<double>::infinity() :
        static_cast<double>(now_ns - last_state_ns) / 1.0e9;
      if (
        mode_.load() != 1 || board_error_code_.load() != 0 ||
        state_age_s > robot_state_timeout_s_)
      {
        RCLCPP_WARN(
          get_logger(),
          "Rejecting trajectory: C board is not healthy/READY/fresh "
          "(mode=%u, error=%u, age=%.3fs)",
          mode_.load(), board_error_code_.load(), state_age_s);
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandle>)
  {
    link_->send_abort_noexcept();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      active_goal_ = goal_handle;
    }
    std::thread(
      [this, goal_handle]() {execute(goal_handle);}).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    try {
      const auto & trajectory = goal_handle->get_goal()->trajectory;
      std::unordered_map<std::string, std::size_t> indices;
      for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
        indices[trajectory.joint_names[index]] = index;
      }
      std::vector<serial::TrajectoryPoint> points;
      points.reserve(trajectory.points.size());
      for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
        const auto & source = trajectory.points[point_index];
        serial::TrajectoryPoint point;
        point.index = static_cast<uint16_t>(point_index);
        point.time_ms = static_cast<uint32_t>(
          static_cast<uint64_t>(source.time_from_start.sec) * 1000ULL +
          source.time_from_start.nanosec / 1000000ULL);
        for (const auto & joint : arm_joints_) {
          const auto index = indices.at(joint);
          point.positions.push_back(source.positions[index]);
          point.velocities.push_back(
            source.velocities.empty() ? 0.0 : source.velocities[index]);
        }
        points.push_back(std::move(point));
      }

      const auto arm_result = link_->send_trajectory(points);
      if (goal_handle->is_canceling()) {
        result->error_code = FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "cancelled by requester";
        goal_handle->canceled(result);
        clear_active_goal();
        return;
      }
      if (arm_result.result_code !=
        static_cast<uint8_t>(serial::ResultCode::kSuccess))
      {
        throw std::runtime_error(
                "C board arm trajectory failed, result=" +
                std::to_string(arm_result.result_code) +
                ", error=" + std::to_string(arm_result.error_code));
      }

      const bool has_gripper =
        indices.count(gripper_joints_[0]) && indices.count(gripper_joints_[1]);
      if (has_gripper) {
        const auto & final = trajectory.points.back().positions;
        const auto left = final[indices.at(gripper_joints_[0])];
        const auto right = final[indices.at(gripper_joints_[1])];
        const auto opening_mm = coupled_jaws_to_opening_mm(left, right);
        const auto gripper_result = link_->send_single_motor_gripper(
          opening_mm, static_cast<uint16_t>(gripper_motor_speed_mm_s_),
          static_cast<uint16_t>(gripper_motor_force_permille_));
        if (gripper_result.result_code !=
          static_cast<uint8_t>(serial::ResultCode::kSuccess))
        {
          throw std::runtime_error(
                  "C board gripper failed, result=" +
                  std::to_string(gripper_result.result_code) +
                  ", error=" + std::to_string(gripper_result.error_code));
        }
      }
      result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
      goal_handle->succeed(result);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Serial execution failed: %s", error.what());
      result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
      result->error_string = error.what();
      if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }
    clear_active_goal();
  }

  void clear_active_goal()
  {
    std::lock_guard<std::mutex> lock(active_mutex_);
    active_goal_.reset();
  }

  void on_robot_state(const serial::RobotState & state)
  {
    if (state.joints.size() != arm_joints_.size() ||
      !std::all_of(
        state.joints.begin(), state.joints.end(),
        [](double value) {return std::isfinite(value);}))
    {
      mode_.store(0);
      board_error_code_.store(0xFFFF);
      last_state_steady_ns_.store(0);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting malformed ROBOT_STATE: expected %zu finite joints, got %zu",
        arm_joints_.size(), state.joints.size());
      return;
    }

    mode_.store(state.mode);
    board_error_code_.store(state.error_code);
    const auto steady_now = std::chrono::steady_clock::now();
    const auto steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      steady_now.time_since_epoch()).count();
    last_state_steady_ns_.store(steady_now_ns);

    std::vector<double> velocities(state.joints.size(), 0.0);
    {
      std::lock_guard<std::mutex> lock(state_history_mutex_);
      if (previous_state_ns_ > 0 && previous_joint_positions_.size() == state.joints.size()) {
        const auto dt = static_cast<double>(steady_now_ns - previous_state_ns_) / 1.0e9;
        if (dt > 1.0e-6) {
          for (std::size_t index = 0; index < state.joints.size(); ++index) {
            velocities[index] =
              (state.joints[index] - previous_joint_positions_[index]) / dt;
          }
        }
      }
      previous_joint_positions_ = state.joints;
      previous_state_ns_ = steady_now_ns;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    const auto arm_count = arm_joints_.size();
    message.name.insert(
      message.name.end(), arm_joints_.begin(), arm_joints_.begin() + arm_count);
    message.position.insert(
      message.position.end(), state.joints.begin(), state.joints.begin() + arm_count);
    message.velocity.insert(
      message.velocity.end(), velocities.begin(), velocities.begin() + arm_count);
    if (state.gripper_opening_mm <= max_opening_mm_) {
      const auto center_offset =
        (static_cast<double>(state.gripper_opening_mm) / 1000.0 +
        finger_thickness_m_) / 2.0;
      message.name.insert(
        message.name.end(), gripper_joints_.begin(), gripper_joints_.end());
      message.position.push_back(center_offset);
      message.position.push_back(-center_offset);
    }
    joint_publisher_->publish(message);

    std::shared_ptr<GoalHandle> active;
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      active = active_goal_;
    }
    if (active && active->is_active()) {
      auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
      feedback->header.stamp = message.header.stamp;
      feedback->joint_names = message.name;
      feedback->actual.positions = message.position;
      active->publish_feedback(feedback);
    }
  }

  void heartbeat()
  {
    if (!link_->running()) {
      RCLCPP_FATAL(
        get_logger(), "Serial link stopped: %s; process will exit for respawn",
        link_->fatal_error().c_str());
      rclcpp::shutdown();
      return;
    }
    try {
      link_->send_heartbeat();
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Serial heartbeat failed: %s", error.what());
    }
  }

  uint16_t coupled_jaws_to_opening_mm(double left, double right) const
  {
    // The URDF has two opposing prismatic joints for collision/visualization,
    // but the real gripper has one motor and a mechanical coupling. Convert
    // the coupled jaw centers into one clear-opening target for that motor.
    if (std::abs(left + right) > gripper_coupling_tolerance_m_) {
      throw std::runtime_error(
              "asymmetric jaw targets cannot be driven by one gripper motor");
    }
    const auto clear_opening_m = std::max(
      0.0, left - right - finger_thickness_m_);
    return static_cast<uint16_t>(std::clamp(
        std::lround(clear_opening_m * 1000.0), 0L,
        static_cast<long>(max_opening_mm_)));
  }

  std::string serial_port_;
  int baudrate_;
  double ack_timeout_;
  int retries_;
  std::string action_name_;
  std::string joint_state_topic_;
  bool require_ready_;
  double robot_state_timeout_s_;
  double heartbeat_rate_;
  std::vector<std::string> arm_joints_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> gripper_lower_limits_;
  std::vector<double> gripper_upper_limits_;
  double finger_thickness_m_;
  double gripper_coupling_tolerance_m_;
  int max_opening_mm_;
  int gripper_motor_speed_mm_s_;
  int gripper_motor_force_permille_;
  std::unique_ptr<serial::SerialLink> link_;
  std::atomic<uint8_t> mode_{0};
  std::atomic<uint16_t> board_error_code_{0xFFFF};
  std::atomic<int64_t> last_state_steady_ns_{0};
  std::mutex state_history_mutex_;
  std::vector<double> previous_joint_positions_;
  int64_t previous_state_ns_{0};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::mutex active_mutex_;
  std::shared_ptr<GoalHandle> active_goal_;
};

}  // namespace jaka_single_arm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<jaka_single_arm::SerialTrajectoryController>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 3);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    std::cerr << "serial_trajectory_controller fatal: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
