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
#include "control_msgs/action/gripper_command.hpp"
#include "fruit_picking_arm/serial_protocol.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

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
  using ClawStateCallback = std::function<void(const ClawState &)>;

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

  void set_claw_state_callback(ClawStateCallback callback)
  {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    claw_state_callback_ = std::move(callback);
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

  MotionResult send_gripper(ClawAction action)
  {
    std::lock_guard<std::mutex> operation_lock(operation_mutex_);
    if (action != ClawAction::kOpen && action != ClawAction::kClose) {
      throw std::runtime_error("gripper action must be OPEN or CLOSE");
    }
    const auto command_sequence = send_message(
      MessageType::kClawCommand,
      {static_cast<uint8_t>(action)}, true);
    return wait_result(command_sequence, 10.0);
  }

  void send_gripper_stop_noexcept()
  {
    try {
      send_message(
        MessageType::kClawCommand,
        {static_cast<uint8_t>(ClawAction::kStop)}, true);
    } catch (...) {
    }
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
    if (frame.type == MessageType::kClawState) {
      ClawState state;
      try {
        state = decode_claw_state(frame.payload);
      } catch (const std::exception &) {
        return;
      }
      ClawStateCallback callback;
      {
        std::lock_guard<std::mutex> lock(callback_mutex_);
        callback = claw_state_callback_;
      }
      if (callback) {
        callback(state);
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
  ClawStateCallback claw_state_callback_;
  mutable std::mutex error_mutex_;
  std::string fatal_error_;
};

}  // namespace serial

class SerialTrajectoryController : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
  using GripperCommand = control_msgs::action::GripperCommand;
  using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperCommand>;

  SerialTrajectoryController()
  : Node("serial_trajectory_controller")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 115200);
    ack_timeout_ = declare_parameter<double>("ack_timeout", 0.25);
    retries_ = declare_parameter<int>("retries", 3);
    action_name_ = declare_parameter<std::string>(
      "action_name", "/arm_controller/follow_joint_trajectory");
    gripper_action_name_ = declare_parameter<std::string>(
      "gripper_action_name", "/gripper_controller/gripper_cmd");
    joint_state_topic_ = declare_parameter<std::string>(
      "joint_state_topic", "/joint_states");
    gripper_state_topic_ = declare_parameter<std::string>(
      "gripper_state_topic", "/gripper/state_estimate");
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
      {-1.483529864, 0.087266463, -3.054326191,
        -2.792526803, -1.483529864, -3.054326191});
    upper_limits_ = declare_parameter<std::vector<double>>(
      "arm_upper_limits",
      {1.483529864, 2.443460952, -0.087266463,
        2.792526803, 1.483529864, 3.054326191});
    feedback_lower_limits_ = declare_parameter<std::vector<double>>(
      "arm_feedback_lower_limits",
      {-1.570796327, 0.0, -3.141592654,
        -2.879793266, -1.570796327, -3.141592654});
    feedback_upper_limits_ = declare_parameter<std::vector<double>>(
      "arm_feedback_upper_limits",
      {1.570796327, 2.530727415, 0.0,
        2.879793266, 1.570796327, 3.141592654});
    feedback_limit_tolerance_ = declare_parameter<double>(
      "feedback_limit_tolerance", 0.02);
    max_velocities_ = declare_parameter<std::vector<double>>(
      "arm_max_velocities", {0.42, 0.50, 0.50, 1.31, 1.50, 1.50});
    max_accelerations_ = declare_parameter<std::vector<double>>(
      "arm_max_accelerations", {0.75, 0.75, 0.75, 0.75, 0.75, 0.75});
    gripper_lower_limits_ = declare_parameter<std::vector<double>>(
      "gripper_lower_limits", {0.0, -0.056});
    gripper_upper_limits_ = declare_parameter<std::vector<double>>(
      "gripper_upper_limits", {0.056, 0.0});
    gripper_initial_positions_ = declare_parameter<std::vector<double>>(
      "unmeasured_gripper_initial_positions", {0.0, 0.0});
    gripper_endpoint_tolerance_m_ = declare_parameter<double>(
      "gripper_endpoint_tolerance_m", 0.002);

    if (arm_joints_.size() != 6 ||
      arm_joints_.size() != lower_limits_.size() ||
      arm_joints_.size() != upper_limits_.size() ||
      arm_joints_.size() != feedback_lower_limits_.size() ||
      arm_joints_.size() != feedback_upper_limits_.size() ||
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
      gripper_upper_limits_.size() != 2 ||
      gripper_initial_positions_.size() != 2)
    {
      throw std::runtime_error("exactly two gripper joints and limits are required");
    }
    for (std::size_t joint = 0; joint < gripper_joints_.size(); ++joint) {
      if (!std::isfinite(gripper_initial_positions_[joint]) ||
        gripper_initial_positions_[joint] < gripper_lower_limits_[joint] ||
        gripper_initial_positions_[joint] > gripper_upper_limits_[joint])
      {
        throw std::runtime_error("unmeasured gripper initial position is outside URDF limits");
      }
    }
    estimated_gripper_positions_ = gripper_initial_positions_;
    if (robot_state_timeout_s_ <= 0.0 || feedback_limit_tolerance_ < 0.0 ||
      !std::isfinite(gripper_endpoint_tolerance_m_) ||
      gripper_endpoint_tolerance_m_ < 0.0 ||
      gripper_endpoint_tolerance_m_ >=
      std::abs(gripper_upper_limits_[0] - gripper_lower_limits_[0]) * 0.5)
    {
      throw std::runtime_error("invalid gripper serial parameters");
    }

    joint_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_state_topic_, 20);
    gripper_state_publisher_ =
      create_publisher<std_msgs::msg::UInt8MultiArray>(gripper_state_topic_, 20);
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
    gripper_action_server_ = rclcpp_action::create_server<GripperCommand>(
      this, gripper_action_name_,
      std::bind(
        &SerialTrajectoryController::handle_gripper_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &SerialTrajectoryController::handle_gripper_cancel, this,
        std::placeholders::_1),
      std::bind(
        &SerialTrajectoryController::handle_gripper_accepted, this,
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
    RCLCPP_WARN(
      get_logger(),
      "CLAW_STATE is open-loop: /joint_states and %s retain/update only an endpoint "
      "estimate; this must never be interpreted as verified fruit contact",
      gripper_state_topic_.c_str());
    publish_gripper_state_estimate(serial::ClawStateCode::kUnknown, false);
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
    if (requested.size() != arm_joints_.size()) {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting mixed arm/gripper trajectory; use %s for the binary gripper",
        gripper_action_name_.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    for (const auto & name : requested) {
      if (std::find(arm_joints_.begin(), arm_joints_.end(), name) == arm_joints_.end()) {
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
        !feedback_in_command_limits_.load() ||
        state_age_s > robot_state_timeout_s_)
      {
        RCLCPP_WARN(
          get_logger(),
          "Rejecting trajectory: C board is not healthy/READY/fresh "
          "(mode=%u, error=%u, feedback_in_command_limits=%s, age=%.3fs)",
          mode_.load(), board_error_code_.load(),
          feedback_in_command_limits_.load() ? "true" : "false", state_age_s);
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

  bool gripper_endpoint_is_open(double position, bool & open) const
  {
    const auto closed_position = gripper_lower_limits_[0];
    const auto open_position = gripper_upper_limits_[0];
    if (!std::isfinite(position)) {
      return false;
    }
    if (std::abs(position - closed_position) <= gripper_endpoint_tolerance_m_) {
      open = false;
      return true;
    }
    if (std::abs(position - open_position) <= gripper_endpoint_tolerance_m_) {
      open = true;
      return true;
    }
    return false;
  }

  rclcpp_action::GoalResponse handle_gripper_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const GripperCommand::Goal> goal)
  {
    bool open = false;
    if (!std::isfinite(goal->command.max_effort) || goal->command.max_effort < 0.0 ||
      !gripper_endpoint_is_open(goal->command.position, open))
    {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting gripper position %.6f m: only %.6f m CLOSED or %.6f m OPEN "
        "are supported (endpoint tolerance %.3f m)",
        goal->command.position, gripper_lower_limits_[0], gripper_upper_limits_[0],
        gripper_endpoint_tolerance_m_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    {
      std::lock_guard<std::mutex> lock(gripper_active_mutex_);
      if (active_gripper_goal_) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
    const auto link = current_link();
    if (!link || !link->running()) {
      RCLCPP_WARN(get_logger(), "Rejecting gripper command: serial is disconnected");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_gripper_cancel(
    const std::shared_ptr<GripperGoalHandle>)
  {
    const auto link = current_link();
    if (link) {
      link->send_gripper_stop_noexcept();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_gripper_accepted(const std::shared_ptr<GripperGoalHandle> goal_handle)
  {
    {
      std::lock_guard<std::mutex> lock(gripper_active_mutex_);
      active_gripper_goal_ = goal_handle;
    }
    std::thread(
      [this, goal_handle]() {execute_gripper(goal_handle);}).detach();
  }

  void execute_gripper(const std::shared_ptr<GripperGoalHandle> goal_handle)
  {
    auto result = std::make_shared<GripperCommand::Result>();
    try {
      bool open = false;
      if (!gripper_endpoint_is_open(goal_handle->get_goal()->command.position, open)) {
        throw std::runtime_error("gripper endpoint changed after goal validation");
      }
      const auto link = current_link();
      if (!link || !link->running()) {
        throw std::runtime_error("serial disconnected before gripper execution");
      }
      const auto command_result = link->send_gripper(
        open ? serial::ClawAction::kOpen : serial::ClawAction::kClose);
      result->position = estimated_gripper_position();
      result->effort = 0.0;
      result->stalled = false;
      if (goal_handle->is_canceling()) {
        result->reached_goal = false;
        goal_handle->canceled(result);
        clear_active_gripper_goal();
        return;
      }
      if (command_result.result_code != static_cast<uint8_t>(serial::ResultCode::kSuccess)) {
        throw std::runtime_error(
                "C board gripper failed, result=" +
                std::to_string(command_result.result_code) +
                ", claw_code=" + std::to_string(command_result.error_code));
      }
      result->reached_goal = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(
        get_logger(),
        "Gripper %s pulse completed; endpoint is an OPEN-LOOP estimate (verified=0)",
        open ? "OPEN" : "CLOSE");
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Serial gripper execution failed: %s", error.what());
      result->position = estimated_gripper_position();
      result->effort = 0.0;
      result->stalled = false;
      result->reached_goal = false;
      if (goal_handle->is_active()) {
        goal_handle->abort(result);
      }
    }
    clear_active_gripper_goal();
  }

  void clear_active_gripper_goal()
  {
    std::lock_guard<std::mutex> lock(gripper_active_mutex_);
    active_gripper_goal_.reset();
  }

  double estimated_gripper_position()
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    return estimated_gripper_positions_.empty() ? 0.0 : estimated_gripper_positions_[0];
  }

  void publish_gripper_state_estimate(serial::ClawStateCode state, bool verified)
  {
    std_msgs::msg::UInt8MultiArray message;
    message.data = {
      static_cast<uint8_t>(state),
      static_cast<uint8_t>(verified ? 1u : 0u)};
    gripper_state_publisher_->publish(message);
  }

  void on_claw_state(const serial::ClawState & state)
  {
    if (state.state_code == serial::ClawStateCode::kOpen) {
      set_estimated_gripper_endpoint(true);
    } else if (state.state_code == serial::ClawStateCode::kClosed) {
      set_estimated_gripper_endpoint(false);
    }
    /* UNKNOWN/OPENING/CLOSING/FAULT deliberately retain the last stable estimate. */
    publish_gripper_state_estimate(state.state_code, state.verified);
  }

  void on_robot_state(const serial::RobotState & state)
  {
    if (state.mode > 4 || state.joint_positions.size() != arm_joints_.size() ||
      !std::all_of(
        state.joint_positions.begin(), state.joint_positions.end(),
        [](double value) {return std::isfinite(value);}))
    {
      mode_.store(0);
      board_error_code_.store(0xFFFF);
      feedback_in_command_limits_.store(false);
      last_state_steady_ns_.store(0);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting malformed ROBOT_STATE: expected %zu finite joints, got %zu",
        arm_joints_.size(), state.joint_positions.size());
      return;
    }

    bool feedback_in_hard_limits = true;
    bool feedback_in_command_limits = true;
    std::size_t first_hard_limit_joint = 0;
    for (std::size_t index = 0; index < state.joint_positions.size(); ++index) {
      const auto position = state.joint_positions[index];
      if (position < feedback_lower_limits_[index] - feedback_limit_tolerance_ ||
        position > feedback_upper_limits_[index] + feedback_limit_tolerance_)
      {
        if (feedback_in_hard_limits) {
          first_hard_limit_joint = index;
        }
        feedback_in_hard_limits = false;
      }
      if (position < lower_limits_[index] || position > upper_limits_[index]) {
        feedback_in_command_limits = false;
      }
    }
    if (!feedback_in_hard_limits) {
      mode_.store(0);
      board_error_code_.store(0xFFFF);
      feedback_in_command_limits_.store(false);
      last_state_steady_ns_.store(0);
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting ROBOT_STATE outside feedback hard limit: %s=%.6f, "
        "allowed=[%.6f, %.6f] plus %.6f rad tolerance",
        arm_joints_[first_hard_limit_joint].c_str(),
        state.joint_positions[first_hard_limit_joint],
        feedback_lower_limits_[first_hard_limit_joint],
        feedback_upper_limits_[first_hard_limit_joint], feedback_limit_tolerance_);
      return;
    }

    feedback_in_command_limits_.store(feedback_in_command_limits);
    if (!feedback_in_command_limits) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ROBOT_STATE is outside MoveIt command soft limits; publishing feedback for "
        "RViz/recovery while trajectory execution remains blocked");
    }

    mode_.store(state.mode);
    board_error_code_.store(state.error_code);
    const auto steady_now = std::chrono::steady_clock::now();
    const auto steady_now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      steady_now.time_since_epoch()).count();
    last_state_steady_ns_.store(steady_now_ns);

    std::vector<double> velocities(state.joint_positions.size(), 0.0);
    {
      std::lock_guard<std::mutex> lock(state_history_mutex_);
      if (
        previous_state_ns_ > 0 &&
        previous_joint_positions_.size() == state.joint_positions.size())
      {
        const auto dt = static_cast<double>(steady_now_ns - previous_state_ns_) / 1.0e9;
        if (dt > 1.0e-6) {
          for (std::size_t index = 0; index < state.joint_positions.size(); ++index) {
            velocities[index] =
              (state.joint_positions[index] - previous_joint_positions_[index]) / dt;
          }
        }
      }
      previous_joint_positions_ = state.joint_positions;
      previous_state_ns_ = steady_now_ns;
    }

    sensor_msgs::msg::JointState message;
    message.header.stamp = now();
    const auto arm_count = arm_joints_.size();
    message.name.insert(
      message.name.end(), arm_joints_.begin(), arm_joints_.begin() + arm_count);
    message.position.insert(
      message.position.end(), state.joint_positions.begin(),
      state.joint_positions.begin() + arm_count);
    message.velocity.insert(
      message.velocity.end(), velocities.begin(), velocities.begin() + arm_count);
    {
      std::lock_guard<std::mutex> lock(gripper_state_mutex_);
      message.name.insert(
        message.name.end(), gripper_joints_.begin(), gripper_joints_.end());
      message.position.insert(
        message.position.end(), estimated_gripper_positions_.begin(),
        estimated_gripper_positions_.end());
      message.velocity.insert(message.velocity.end(), gripper_joints_.size(), 0.0);
    }
    joint_publisher_->publish(message);
    if (!robot_state_ready_logged_.exchange(true)) {
      RCLCPP_INFO(
        get_logger(),
        "REAL_ROBOT_STATE_READY: valid six-axis control-board state published");
    }

    std::shared_ptr<GoalHandle> active;
    {
      std::lock_guard<std::mutex> lock(active_mutex_);
      active = active_goal_;
    }
    if (active && active->is_active()) {
      auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
      feedback->header.stamp = message.header.stamp;
      /* Keep the six-axis action contract separate from the two estimated
       * gripper joints that are appended only to /joint_states. */
      feedback->joint_names = arm_joints_;
      feedback->actual.positions = state.joint_positions;
      feedback->actual.velocities = velocities;
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
      candidate->set_claw_state_callback(
        [this](const serial::ClawState & state) {on_claw_state(state);});
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
    publish_gripper_state_estimate(serial::ClawStateCode::kUnknown, false);
    RCLCPP_WARN(
      get_logger(), "Serial connection lost: %s; motion blocked while reconnecting",
      reason.c_str());
  }

  void set_estimated_gripper_endpoint(bool open)
  {
    std::lock_guard<std::mutex> lock(gripper_state_mutex_);
    if (open) {
      estimated_gripper_positions_ = {
        gripper_upper_limits_[0], gripper_lower_limits_[1]};
    } else {
      estimated_gripper_positions_ = {
        gripper_lower_limits_[0], gripper_upper_limits_[1]};
    }
  }

  std::string serial_port_;
  int baudrate_;
  double ack_timeout_;
  int retries_;
  std::string action_name_;
  std::string gripper_action_name_;
  std::string joint_state_topic_;
  std::string gripper_state_topic_;
  bool require_ready_;
  double robot_state_timeout_s_;
  double heartbeat_rate_;
  double reconnect_interval_s_;
  std::vector<std::string> arm_joints_;
  std::vector<std::string> gripper_joints_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::vector<double> feedback_lower_limits_;
  std::vector<double> feedback_upper_limits_;
  double feedback_limit_tolerance_;
  std::vector<double> max_velocities_;
  std::vector<double> max_accelerations_;
  std::vector<double> gripper_lower_limits_;
  std::vector<double> gripper_upper_limits_;
  std::vector<double> gripper_initial_positions_;
  double gripper_endpoint_tolerance_m_;
  std::mutex gripper_state_mutex_;
  std::vector<double> estimated_gripper_positions_;
  mutable std::mutex link_mutex_;
  std::shared_ptr<serial::SerialLink> link_;
  std::chrono::steady_clock::time_point last_connect_attempt_{};
  std::atomic<uint8_t> mode_{0};
  std::atomic<uint16_t> board_error_code_{0xFFFF};
  std::atomic<bool> feedback_in_command_limits_{false};
  std::atomic<bool> robot_state_ready_logged_{false};
  std::atomic<int64_t> last_state_steady_ns_{0};
  std::mutex state_history_mutex_;
  std::vector<double> previous_joint_positions_;
  int64_t previous_state_ns_{0};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr gripper_state_publisher_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
  rclcpp_action::Server<GripperCommand>::SharedPtr gripper_action_server_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;
  std::mutex active_mutex_;
  std::shared_ptr<GoalHandle> active_goal_;
  std::mutex gripper_active_mutex_;
  std::shared_ptr<GripperGoalHandle> active_gripper_goal_;
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
