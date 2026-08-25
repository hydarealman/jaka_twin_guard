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
#include "fruit_picking_arm/serial_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace fruit_picking_arm
{
namespace serial
{

speed_t baud_constant(int baudrate)
{
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: throw std::runtime_error("unsupported baud rate: " + std::to_string(baudrate));
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
    retries_(std::max(0, retries))
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
    const auto packet = encode_frame(Frame{type, sequence, payload});
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
                    ", status=" + std::to_string(pending->status));
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
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (points.size() > kMaxTrajectoryPoints) {
      throw std::runtime_error("trajectory exceeds C-board 100-point cache");
    }
    constexpr std::size_t expected_joint_count = kJointCount;
    if (points.front().positions.size() != expected_joint_count ||
      points.front().velocities.size() != expected_joint_count)
    {
      throw std::runtime_error("trajectory must contain six positions and velocities");
    }
    std::uint32_t previous_time_ms = 0;
    bool first_point = true;
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto & point = points[index];
      if (point.index != index || point.positions.size() != expected_joint_count ||
        point.velocities.size() != expected_joint_count ||
        (!first_point && point.time_ms <= previous_time_ms))
      {
        throw std::runtime_error("trajectory points are not contiguous or strictly timed");
      }
      for (const auto value : point.positions) {
        (void)encode_angle(value);
      }
      for (const auto value : point.velocities) {
        (void)encode_angle(value);
      }
      previous_time_ms = point.time_ms;
      first_point = false;
    }
    std::vector<uint8_t> begin;
    append_u16(begin, static_cast<uint16_t>(points.size()));
    send_message(MessageType::kTrajectoryBegin, begin, true);

    std::uint16_t end_sequence = 0;
    try {
      for (const auto & point : points) {
        std::vector<uint8_t> payload;
        append_u16(payload, point.index);
        append_u32(payload, point.time_ms);
        for (const auto position : point.positions) {
          append_i32(payload, encode_angle(position));
        }
        for (const auto velocity : point.velocities) {
          append_i32(payload, encode_angle(velocity));
        }
        send_message(MessageType::kTrajectoryPoint, payload, true);
      }
      end_sequence = send_message(MessageType::kTrajectoryEnd, {}, true);
    } catch (...) {
      send_abort_noexcept();
      throw;
    }
    const auto timeout = std::max(
      10.0, static_cast<double>(points.back().time_ms) / 1000.0 + 10.0);
    return wait_result(end_sequence, timeout);
  }

  MotionResult send_single_motor_gripper(uint16_t opening_mm)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (opening_mm > 100) {
      throw std::runtime_error("gripper command is outside protocol limits");
    }
    const auto action = opening_mm == 0 ? ClawAction::kClose : ClawAction::kOpen;
    const auto command_sequence = send_message(
      MessageType::kClawCommand,
      {static_cast<uint8_t>(action)}, true);
    return wait_result(command_sequence, 10.0);
  }

  void send_heartbeat()
  {
    send_message(MessageType::kHeartbeat, {}, false);
  }

  void send_abort_noexcept()
  {
    try {
      send_message(MessageType::kStop, {}, true);
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
  };

  uint16_t next_sequence()
  {
    auto next = ++sequence_;
    if (next == 0) {
      next = ++sequence_;
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
    if (frame.type == MessageType::kMotionDone || frame.type == MessageType::kClawResult) {
      const auto packet = encode_frame(Frame{MessageType::kAck, frame.sequence, {0}});
      write_packet(packet);
    }
    if (frame.type == MessageType::kAck) {
      if (frame.payload.size() != 1) {
        return;
      }
      std::shared_ptr<PendingAck> pending;
      {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        const auto found = pending_.find(frame.sequence);
        if (found != pending_.end()) {
          pending = found->second;
        }
      }
      if (pending) {
        {
          std::lock_guard<std::mutex> lock(pending->mutex);
          pending->status = frame.payload[0];
          pending->done = true;
        }
        pending->condition.notify_all();
      }
      return;
    }
    if (frame.type == MessageType::kMotionDone) {
      try {
        auto result = decode_motion_result(frame.payload);
        result.command_sequence = frame.sequence;
        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          const auto duplicate = std::find_if(
            results_.begin(), results_.end(),
            [&result](const MotionResult & queued) {
              return queued.command_sequence == result.command_sequence;
            });
          if (duplicate == results_.end()) {
            results_.push_back(result);
          }
          if (results_.size() > 128) {
            results_.pop_front();
          }
        }
        result_condition_.notify_all();
      } catch (const std::exception &) {
        return;
      }
      return;
    }
    if (frame.type == MessageType::kClawResult) {
      try {
        const auto claw = decode_claw_result(frame.payload);
        const bool success = claw.result_code == ClawResultCode::kCompletedUnverified;
        {
          std::lock_guard<std::mutex> lock(result_mutex_);
          results_.push_back(MotionResult{
            frame.sequence,
            static_cast<uint8_t>(success ? ResultCode::kSuccess : ResultCode::kFailed),
            static_cast<uint16_t>(claw.result_code)});
        }
        result_condition_.notify_all();
      } catch (const std::exception &) {
      }
      return;
    }
    if (frame.type == MessageType::kRobotState) {
      RobotState state;
      try {
        state = decode_robot_state(frame.payload);
      } catch (const std::exception &) {
        return;
      }
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

  MotionResult wait_result(std::uint16_t expected_sequence, double timeout_seconds)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(timeout_seconds);
    std::unique_lock<std::mutex> lock(result_mutex_);
    while (running()) {
      const auto found = std::find_if(
        results_.begin(), results_.end(),
        [expected_sequence](const MotionResult & result) {
          return result.command_sequence == expected_sequence;
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
    throw std::runtime_error("motion result timeout");
  }

  PosixSerialPort port_;
  std::chrono::duration<double> ack_timeout_;
  int retries_;
  std::atomic<bool> running_{false};
  std::atomic<uint16_t> sequence_{0};
  std::thread reader_;
  FrameParser parser_;
  std::mutex write_mutex_;
  std::mutex operation_mutex_;
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
    heartbeat_rate_ = declare_parameter<double>("heartbeat_rate", 10.0);
    reconnect_interval_s_ = declare_parameter<double>("reconnect_interval", 2.0);
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
    max_velocities_ = declare_parameter<std::vector<double>>(
      "arm_max_velocities", {0.42, 0.50, 0.50, 1.31, 1.50, 1.50});
    max_accelerations_ = declare_parameter<std::vector<double>>(
      "arm_max_accelerations", {0.75, 0.75, 0.75, 0.75, 0.75, 0.75});
    gripper_lower_limits_ = declare_parameter<std::vector<double>>(
      "gripper_lower_limits", {0.0, -0.056});
    gripper_upper_limits_ = declare_parameter<std::vector<double>>(
      "gripper_upper_limits", {0.056, 0.0});
    finger_thickness_m_ = declare_parameter<double>(
      "gripper_finger_thickness_m", 0.012);
    gripper_coupling_tolerance_m_ = declare_parameter<double>(
      "gripper_coupling_tolerance_m", 0.0005);
    max_opening_mm_ = declare_parameter<int>("gripper_max_opening_mm", 100);

    if (arm_joints_.size() != 6 ||
      arm_joints_.size() != lower_limits_.size() ||
      arm_joints_.size() != upper_limits_.size() ||
      arm_joints_.size() != max_velocities_.size() ||
      arm_joints_.size() != max_accelerations_.size())
    {
      throw std::runtime_error("exactly six arm joints and six arm limits are required");
    }
    for (std::size_t joint = 0; joint < arm_joints_.size(); ++joint) {
      if (max_velocities_[joint] <= 0.0 || max_accelerations_[joint] <= 0.0) {
        throw std::runtime_error("arm velocity and acceleration limits must be positive");
      }
    }
    if (gripper_joints_.size() != 2 ||
      gripper_lower_limits_.size() != 2 ||
      gripper_upper_limits_.size() != 2)
    {
      throw std::runtime_error("exactly two gripper joints and limits are required");
    }
    if (robot_state_timeout_s_ <= 0.0 || gripper_coupling_tolerance_m_ < 0.0 ||
      max_opening_mm_ < 1 || max_opening_mm_ > 100)
    {
      throw std::runtime_error("invalid gripper serial parameters");
    }

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
      "Serial controller started: %s @ %d; motion blocked until fresh READY",
      serial_port_.c_str(), baudrate_);
    try_connect();
  }

  ~SerialTrajectoryController() override
  {
    const auto link = current_link();
    if (link) {
      link->close();
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
    if (trajectory.points.size() > serial::kMaxTrajectoryPoints) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory exceeding C-board 100-point cache");
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
    std::uint64_t previous_time_ms = 0;
    std::vector<double> previous_positions(arm_joints_.size(), 0.0);
    std::vector<double> previous_segment_velocity(arm_joints_.size(), 0.0);
    for (std::size_t point_index = 0; point_index < trajectory.points.size(); ++point_index) {
      const auto & point = trajectory.points[point_index];
      if (point.positions.size() != trajectory.joint_names.size() ||
        point.velocities.size() != trajectory.joint_names.size())
      {
        RCLCPP_WARN(get_logger(), "Rejecting malformed trajectory point");
        return rclcpp_action::GoalResponse::REJECT;
      }
      const auto time_ns =
        static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
        point.time_from_start.nanosec;
      if (time_ns < 0 || time_ns <= previous_ns) {
        RCLCPP_WARN(get_logger(), "Rejecting non-increasing trajectory time");
        return rclcpp_action::GoalResponse::REJECT;
      }
      previous_ns = time_ns;
      const auto time_ms = static_cast<std::uint64_t>(point.time_from_start.sec) * 1000ULL +
        point.time_from_start.nanosec / 1000000ULL;
      if (time_ms > UINT32_MAX ||
        (point_index > 0 && time_ms <= previous_time_ms))
      {
        RCLCPP_WARN(get_logger(), "Rejecting trajectory with invalid millisecond timing");
        return rclcpp_action::GoalResponse::REJECT;
      }
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
        const auto velocity = point.velocities[indices.at(arm_joints_[joint])];
        if (!std::isfinite(velocity) || std::abs(velocity) > max_velocities_[joint]) {
          RCLCPP_ERROR(
            get_logger(), "Rejecting unsafe %s velocity %.6f at point %zu",
            arm_joints_[joint].c_str(), velocity, point_index);
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (point_index > 0) {
          const auto dt = static_cast<double>(time_ms - previous_time_ms) / 1000.0;
          const auto segment_velocity = (value - previous_positions[joint]) / dt;
          if (std::abs(segment_velocity) > max_velocities_[joint]) {
            RCLCPP_ERROR(
              get_logger(), "Rejecting %s segment speed %.6f at point %zu",
              arm_joints_[joint].c_str(), segment_velocity, point_index);
            return rclcpp_action::GoalResponse::REJECT;
          }
          if (point_index > 1 &&
            std::abs(segment_velocity - previous_segment_velocity[joint]) / dt >
            max_accelerations_[joint])
          {
            RCLCPP_ERROR(
              get_logger(), "Rejecting %s segment acceleration at point %zu",
              arm_joints_[joint].c_str(), point_index);
            return rclcpp_action::GoalResponse::REJECT;
          }
          previous_segment_velocity[joint] = segment_velocity;
        }
        previous_positions[joint] = value;
        try {
          (void)serial::encode_angle(value);
        } catch (const std::exception &) {
          RCLCPP_ERROR(get_logger(), "Rejecting arm angle outside wire range");
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
      for (const auto velocity : point.velocities) {
        if (!std::isfinite(velocity)) {
          RCLCPP_ERROR(get_logger(), "Rejecting non-finite trajectory velocity");
          return rclcpp_action::GoalResponse::REJECT;
        }
        try {
          (void)serial::encode_angle(velocity);
        } catch (const std::exception &) {
          RCLCPP_ERROR(get_logger(), "Rejecting velocity outside wire range");
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
      previous_time_ms = time_ms;
    }
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      if (active_goal_) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    const auto link = current_link();
    if (!link || !link->running()) {
      RCLCPP_WARN(get_logger(), "Rejecting trajectory: serial is disconnected");
      return rclcpp_action::GoalResponse::REJECT;
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
    const auto link = current_link();
    if (link) {
      link->send_abort_noexcept();
    }
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
      const auto link = current_link();
      if (!link || !link->running()) {
        throw std::runtime_error("serial disconnected before execution");
      }
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
          point.velocities.push_back(source.velocities[index]);
        }
        points.push_back(std::move(point));
      }

      const auto arm_result = link->send_trajectory(points);
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
        const auto gripper_result = link->send_single_motor_gripper(opening_mm);
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
    bool joints_in_limits = state.joints.size() == arm_joints_.size();
    for (std::size_t index = 0; joints_in_limits && index < state.joints.size(); ++index) {
      joints_in_limits =
        state.joints[index] >= lower_limits_[index] &&
        state.joints[index] <= upper_limits_[index];
    }
    if (state.mode > 4 || state.joints.size() != arm_joints_.size() ||
      !std::all_of(
        state.joints.begin(), state.joints.end(),
        [](double value) {return std::isfinite(value);}) ||
      !joints_in_limits)
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
    auto link = current_link();
    if (!link) {
      try_connect();
      return;
    }
    if (!link->running()) {
      drop_link(link, link->fatal_error());
      try_connect();
      return;
    }
    try {
      link->send_heartbeat();
    } catch (const std::exception & error) {
      drop_link(link, error.what());
    }
  }

  std::shared_ptr<serial::SerialLink> current_link() const
  {
    std::lock_guard<std::mutex> lock(link_mutex_);
    return link_;
  }

  void try_connect()
  {
    if (current_link()) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (last_connect_attempt_.time_since_epoch().count() != 0 &&
      std::chrono::duration<double>(now - last_connect_attempt_).count() <
      reconnect_interval_s_)
    {
      return;
    }
    last_connect_attempt_ = now;
    try {
      auto candidate = std::make_shared<serial::SerialLink>(
        serial_port_, baudrate_, ack_timeout_, retries_);
      candidate->set_state_callback(
        [this](const serial::RobotState & state) {on_robot_state(state);});
      {
        std::lock_guard<std::mutex> lock(link_mutex_);
        if (link_) {
          candidate->close();
          return;
        }
        link_ = candidate;
      }
      mode_.store(0);
      board_error_code_.store(0xFFFF);
      last_state_steady_ns_.store(0);
      RCLCPP_INFO(
        get_logger(), "Serial connected; waiting for fresh C-board READY state");
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Serial unavailable: %s; retrying in %.1fs",
        error.what(), reconnect_interval_s_);
    }
  }

  void drop_link(
    const std::shared_ptr<serial::SerialLink> & failed,
    const std::string & reason)
  {
    {
      std::lock_guard<std::mutex> lock(link_mutex_);
      if (link_ != failed) {
        return;
      }
      link_.reset();
    }
    failed->send_abort_noexcept();
    failed->close();
    mode_.store(0);
    board_error_code_.store(0xFFFF);
    last_state_steady_ns_.store(0);
    RCLCPP_WARN(
      get_logger(), "Serial connection lost: %s; motion blocked while reconnecting",
      reason.c_str());
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
  double reconnect_interval_s_;
  std::vector<std::string> arm_joints_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;
  std::vector<double> gripper_lower_limits_;
  std::vector<double> gripper_upper_limits_;
  double finger_thickness_m_;
  double gripper_coupling_tolerance_m_;
  int max_opening_mm_;
  mutable std::mutex link_mutex_;
  std::shared_ptr<serial::SerialLink> link_;
  std::chrono::steady_clock::time_point last_connect_attempt_{};
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

}  // namespace fruit_picking_arm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<fruit_picking_arm::SerialTrajectoryController>();
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
