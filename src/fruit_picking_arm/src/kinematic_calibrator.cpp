#include "fruit_picking_arm/calibration_target_detector.hpp"
#include "fruit_picking_arm/kinematic_calibration.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fruit_picking_arm::calibration
{
namespace
{

using Trigger = std_srvs::srv::Trigger;

cv::Mat camera_matrix_from(const sensor_msgs::msg::CameraInfo & info)
{
  return (cv::Mat_<double>(3, 3) <<
    info.k[0], info.k[1], info.k[2],
    info.k[3], info.k[4], info.k[5],
    info.k[6], info.k[7], info.k[8]);
}

cv::Mat distortion_from(const sensor_msgs::msg::CameraInfo & info)
{
  cv::Mat distortion(1, static_cast<int>(info.d.size()), CV_64F);
  for (std::size_t index = 0; index < info.d.size(); ++index) {
    distortion.at<double>(0, static_cast<int>(index)) = info.d[index];
  }
  return distortion;
}

YAML::Node transform_node(const Transform3d & transform)
{
  YAML::Node node;
  std::vector<double> rotation;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation.push_back(transform.rotation(row, column));
    }
  }
  node["rotation_row_major"] = rotation;
  node["translation_m"] = std::vector<double>{
    transform.translation[0], transform.translation[1], transform.translation[2]};
  return node;
}

Transform3d transform_from_message(const geometry_msgs::msg::Transform & message)
{
  tf2::Quaternion quaternion(
    message.rotation.x, message.rotation.y, message.rotation.z, message.rotation.w);
  if (quaternion.length2() < 1.0e-12) {
    throw std::runtime_error("TF contains an invalid quaternion");
  }
  quaternion.normalize();
  tf2::Matrix3x3 matrix(quaternion);
  Transform3d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result.rotation(row, column) = matrix[row][column];
    }
  }
  result.translation = {
    message.translation.x, message.translation.y, message.translation.z};
  return result;
}

}  // namespace

class KinematicCalibrator : public rclcpp::Node
{
public:
  KinematicCalibrator()
  : Node("kinematic_calibrator"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_serial_ = declare_parameter<std::string>("robot_serial", "");
    output_yaml_ = declare_parameter<std::string>("output_yaml", "kinematics.yaml");
    sample_directory_ = declare_parameter<std::string>("sample_directory", "");
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/camera/camera/color/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/camera/camera/color/camera_info");
    joint_state_topic_ = declare_parameter<std::string>("joint_state_topic", "/joint_states");
    annotated_topic_ = declare_parameter<std::string>(
      "annotated_topic", "/kinematic_calibration/annotated");
    camera_frame_ = declare_parameter<std::string>(
      "camera_frame", "camera_color_optical_frame");
    camera_root_frame_ = declare_parameter<std::string>("camera_root_frame", "camera_link");
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    max_reprojection_error_px_ = declare_parameter("max_reprojection_error_px", 1.5);
    max_sample_age_s_ = declare_parameter("max_sample_age_s", 0.75);
    max_sync_delta_s_ = declare_parameter("max_sync_delta_s", 0.08);
    stationary_velocity_rad_s_ = declare_parameter("stationary_velocity_rad_s", 0.01);
    duplicate_joint_distance_rad_ = declare_parameter("duplicate_joint_distance_rad", 0.05);

    KinematicCalibrationOptions options;
    options.minimum_training_samples = static_cast<std::size_t>(
      declare_parameter("minimum_training_samples", 30));
    options.minimum_validation_samples = static_cast<std::size_t>(
      declare_parameter("minimum_validation_samples", 10));
    options.translation_sigma_m = declare_parameter("translation_sigma_m", 0.003);
    options.rotation_sigma_rad = declare_parameter("rotation_sigma_rad", 0.008726646259971648);
    options.robust_loss_scale = declare_parameter("robust_loss_scale", 2.0);
    options.maximum_joint_offset_rad = declare_parameter("maximum_joint_offset_rad", 0.15);
    options.maximum_link_correction_m = declare_parameter("maximum_link_correction_m", 0.03);
    options.minimum_joint_span_rad = declare_parameter("minimum_joint_span_rad", 0.15);
    options.maximum_condition_number = declare_parameter("maximum_condition_number", 1.0e10);
    options.validation_position_p95_m = declare_parameter(
      "validation_position_p95_m", 0.005);
    options.validation_rotation_p95_deg = declare_parameter(
      "validation_rotation_p95_deg", 1.0);
    options_ = options;

    CalibrationTargetSettings target;
    target.target_type = declare_parameter<std::string>("target_type", "chessboard");
    target.dictionary = declare_parameter<std::string>("dictionary", "DICT_5X5_250");
    target.charuco_squares_x = declare_parameter("charuco_squares_x", 7);
    target.charuco_squares_y = declare_parameter("charuco_squares_y", 5);
    target.chessboard_corners_x = declare_parameter("chessboard_corners_x", 9);
    target.chessboard_corners_y = declare_parameter("chessboard_corners_y", 6);
    target.square_length_m = declare_parameter("square_length_m", 0.030);
    target.marker_length_m = declare_parameter("marker_length_m", 0.022);
    target.minimum_detected_corners = declare_parameter("min_detected_corners", 8);
    detector_ = std::make_unique<CalibrationTargetDetector>(target);

    if (robot_serial_.empty()) {
      throw std::invalid_argument("robot_serial is required for kinematic calibration");
    }
    if (joint_names_.size() != 6) {
      throw std::invalid_argument("exactly six joint_names are required");
    }

    const auto qos = rclcpp::SensorDataQoS();
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, qos,
      std::bind(&KinematicCalibrator::on_image, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, qos,
      std::bind(&KinematicCalibrator::on_camera_info, this, std::placeholders::_1));
    joint_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, qos,
      std::bind(&KinematicCalibrator::on_joint_state, this, std::placeholders::_1));
    annotated_publisher_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, 2);

    capture_service_ = make_service(
      "/kinematic_calibration/capture", &KinematicCalibrator::capture);
    remove_service_ = make_service(
      "/kinematic_calibration/remove_last", &KinematicCalibrator::remove_last);
    clear_service_ = make_service(
      "/kinematic_calibration/clear", &KinematicCalibrator::clear);
    solve_service_ = make_service(
      "/kinematic_calibration/solve", &KinematicCalibrator::solve);
    save_service_ = make_service(
      "/kinematic_calibration/save", &KinematicCalibrator::save);
    status_service_ = make_service(
      "/kinematic_calibration/status", &KinematicCalibrator::status);

    RCLCPP_INFO(
      get_logger(),
      "C++ kinematic calibrator ready for robot '%s'; output=%s",
      robot_serial_.c_str(), output_yaml_.c_str());
    RCLCPP_WARN(
      get_logger(),
      "Manual low-speed jogging only. J1/J6 offsets remain fixed to mechanical zero because they are gauge freedoms in this camera/board setup.");
  }

private:
  struct TimedJointState
  {
    rclcpp::Time stamp;
    std::array<double, 6> positions{};
    std::array<double, 6> velocities{};
  };

  struct SynchronizedObservation
  {
    KinematicSample sample;
    rclcpp::Time image_stamp;
    rclcpp::Time joint_stamp;
    cv::Mat image;
  };

  template<typename Callback>
  rclcpp::Service<Trigger>::SharedPtr make_service(const std::string & name, Callback callback)
  {
    return create_service<Trigger>(
      name, std::bind(callback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    if (message->k[0] <= 0.0 || message->k[4] <= 0.0) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    camera_info_ = *message;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr message)
  {
    TimedJointState state;
    state.stamp = rclcpp::Time(message->header.stamp);
    if (state.stamp.nanoseconds() == 0) {state.stamp = now();}
    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
      const auto found = std::find(message->name.begin(), message->name.end(), joint_names_[joint]);
      if (found == message->name.end()) {return;}
      const auto index = static_cast<std::size_t>(std::distance(message->name.begin(), found));
      if (index >= message->position.size() || !std::isfinite(message->position[index])) {return;}
      state.positions[joint] = message->position[index];
      state.velocities[joint] = index < message->velocity.size() ? message->velocity[index] :
        std::numeric_limits<double>::infinity();
    }
    std::lock_guard<std::mutex> lock(mutex_);
    joint_history_.push_back(state);
    while (!joint_history_.empty() &&
      (state.stamp - joint_history_.front().stamp).seconds() > 2.0)
    {
      joint_history_.pop_front();
    }
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    sensor_msgs::msg::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!camera_info_) {return;}
      camera_info = *camera_info_;
    }
    cv_bridge::CvImageConstPtr converted;
    try {
      converted = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", error.what());
      return;
    }
    const auto detection = detector_->detect(
      converted->image, camera_matrix_from(camera_info), distortion_from(camera_info));
    if (!detection) {return;}

    auto annotated = cv_bridge::CvImage(
      message->header, sensor_msgs::image_encodings::BGR8,
      detection->annotated_image).toImageMsg();
    annotated_publisher_->publish(*annotated);

    const auto image_stamp = rclcpp::Time(message->header.stamp);
    std::lock_guard<std::mutex> lock(mutex_);
    if (joint_history_.empty()) {return;}
    const auto nearest = std::min_element(
      joint_history_.begin(), joint_history_.end(),
      [&image_stamp](const auto & lhs, const auto & rhs) {
        return std::abs((lhs.stamp - image_stamp).seconds()) <
               std::abs((rhs.stamp - image_stamp).seconds());
      });
    if (std::abs((nearest->stamp - image_stamp).seconds()) > max_sync_delta_s_) {return;}
    latest_ = SynchronizedObservation{
      {nearest->positions, detection->camera_T_target, detection->reprojection_error_px},
      image_stamp, nearest->stamp, converted->image.clone()};
    latest_velocities_ = nearest->velocities;
  }

  void capture(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_) {return fail(response, "no synchronized board/joint observation");}
    const auto age = (now() - latest_->image_stamp).seconds();
    if (age < 0.0 || age > max_sample_age_s_) {return fail(response, "observation is stale");}
    if (latest_->sample.reprojection_error_px > max_reprojection_error_px_) {
      return fail(response, "board reprojection error exceeds limit");
    }
    if (std::any_of(
        latest_velocities_.begin(), latest_velocities_.end(),
        [this](double velocity) {
          return !std::isfinite(velocity) || std::abs(velocity) > stationary_velocity_rad_s_;
        }))
    {
      return fail(response, "robot is moving or joint velocity is unavailable");
    }
    if (!samples_.empty()) {
      double distance_squared = 0.0;
      for (std::size_t joint = 0; joint < 6; ++joint) {
        const auto delta = latest_->sample.joints[joint] - samples_.back().sample.joints[joint];
        distance_squared += delta * delta;
      }
      if (std::sqrt(distance_squared) < duplicate_joint_distance_rad_) {
        return fail(response, "joint pose is too similar to the previous sample");
      }
    }
    samples_.push_back(*latest_);
    result_.reset();
    response->success = true;
    response->message = "captured sample " + std::to_string(samples_.size()) +
      ", reprojection=" + std::to_string(latest_->sample.reprojection_error_px) + " px";
  }

  void remove_last(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) {return fail(response, "no samples to remove");}
    samples_.pop_back();
    result_.reset();
    response->success = true;
    response->message = "removed last sample; remaining=" + std::to_string(samples_.size());
  }

  void clear(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
    result_.reset();
    response->success = true;
    response->message = "cleared all kinematic calibration samples";
  }

  void solve(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::vector<KinematicSample> samples;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & sample : samples_) {samples.push_back(sample.sample);}
    }
    try {
      const auto result = solve_kinematic_calibration(samples, options_);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = result;
      }
      response->success = result.validated;
      std::ostringstream message;
      message << "solved: train P95=" << result.training_metrics.position_p95_m * 1000.0
              << " mm/" << result.training_metrics.rotation_p95_deg
              << " deg, validation P95=" << result.validation_metrics.position_p95_m * 1000.0
              << " mm/" << result.validation_metrics.rotation_p95_deg
              << " deg, condition=" << result.jacobian_condition_number
              << ", validated=" << (result.validated ? "true" : "false");
      response->message = message.str();
    } catch (const std::exception & error) {
      fail(response, std::string("solve failed: ") + error.what());
    }
  }

  void save(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::optional<KinematicCalibrationResult> result;
    std::vector<SynchronizedObservation> samples;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = result_;
      samples = samples_;
    }
    if (!result) {return fail(response, "solve before saving");}
    try {
      auto profile_result = *result;
      if (camera_root_frame_ != camera_frame_) {
        const auto root_T_optical = tf_buffer_.lookupTransform(
          camera_root_frame_, camera_frame_, tf2::TimePointZero);
        profile_result.base_T_camera = compose(
          result->base_T_camera,
          inverse(transform_from_message(root_T_optical.transform)));
      }
      write_kinematic_profile(output_yaml_, robot_serial_, profile_result);
      save_samples(samples);
      response->success = result->validated;
      response->message = std::string(result->validated ? "saved validated profile to " :
        "saved UNVALIDATED diagnostic profile to ") + output_yaml_;
    } catch (const std::exception & error) {
      fail(response, std::string("save failed: ") + error.what());
    }
  }

  void status(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = true;
    response->message = "samples=" + std::to_string(samples_.size()) +
      ", required=" + std::to_string(
      options_.minimum_training_samples + options_.minimum_validation_samples) +
      ", solved=" + (result_ ? "true" : "false") +
      ", validated=" + (result_ && result_->validated ? "true" : "false");
  }

  void save_samples(const std::vector<SynchronizedObservation> & samples)
  {
    auto directory = sample_directory_.empty() ?
      std::filesystem::path(output_yaml_).parent_path() / "samples" :
      std::filesystem::path(sample_directory_);
    std::filesystem::create_directories(directory);
    YAML::Node root;
    root["schema_version"] = 1;
    root["robot_serial"] = robot_serial_;
    root["nominal_model_hash"] = kNominalModelHash;
    for (std::size_t index = 0; index < samples.size(); ++index) {
      YAML::Node sample;
      sample["index"] = index;
      sample["split"] = index % 4U == 3U ? "validation" : "training";
      sample["joint_positions_rad"] = std::vector<double>(
        samples[index].sample.joints.begin(), samples[index].sample.joints.end());
      sample["camera_T_board"] = transform_node(samples[index].sample.camera_T_board);
      sample["reprojection_error_px"] = samples[index].sample.reprojection_error_px;
      const auto image_name = "sample_" + std::to_string(index) + ".png";
      sample["image"] = image_name;
      root["samples"].push_back(sample);
      cv::imwrite((directory / image_name).string(), samples[index].image);
    }
    std::ofstream output(directory / "dataset.yaml", std::ios::trunc);
    if (!output.is_open()) {throw std::runtime_error("cannot write sample dataset");}
    output << root;
  }

  static void fail(const std::shared_ptr<Trigger::Response> & response, const std::string & message)
  {
    response->success = false;
    response->message = message;
  }

  std::string robot_serial_;
  std::string output_yaml_;
  std::string sample_directory_;
  std::string image_topic_;
  std::string camera_info_topic_;
  std::string joint_state_topic_;
  std::string annotated_topic_;
  std::string camera_frame_;
  std::string camera_root_frame_;
  std::vector<std::string> joint_names_;
  double max_reprojection_error_px_{1.5};
  double max_sample_age_s_{0.75};
  double max_sync_delta_s_{0.08};
  double stationary_velocity_rad_s_{0.01};
  double duplicate_joint_distance_rad_{0.05};
  KinematicCalibrationOptions options_;
  std::unique_ptr<CalibrationTargetDetector> detector_;

  std::mutex mutex_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;
  std::deque<TimedJointState> joint_history_;
  std::optional<SynchronizedObservation> latest_;
  std::array<double, 6> latest_velocities_{};
  std::vector<SynchronizedObservation> samples_;
  std::optional<KinematicCalibrationResult> result_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_publisher_;
  rclcpp::Service<Trigger>::SharedPtr capture_service_;
  rclcpp::Service<Trigger>::SharedPtr remove_service_;
  rclcpp::Service<Trigger>::SharedPtr clear_service_;
  rclcpp::Service<Trigger>::SharedPtr solve_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr status_service_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace fruit_picking_arm::calibration

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fruit_picking_arm::calibration::KinematicCalibrator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("kinematic_calibrator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
