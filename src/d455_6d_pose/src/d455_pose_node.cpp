#include "d455_6d_pose/pose_core.hpp"

#include <cv_bridge/cv_bridge.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace d455_6d_pose
{
namespace
{

using Image = sensor_msgs::msg::Image;
using ApproximatePolicy = message_filters::sync_policies::ApproximateTime<Image, Image>;

diagnostic_msgs::msg::KeyValue key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

std::string number(double value, int precision = 4)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

geometry_msgs::msg::Quaternion quaternion_from_rotation(const cv::Matx33d & rotation)
{
  tf2::Matrix3x3 matrix(
    rotation(0, 0), rotation(0, 1), rotation(0, 2),
    rotation(1, 0), rotation(1, 1), rotation(1, 2),
    rotation(2, 0), rotation(2, 1), rotation(2, 2));
  tf2::Quaternion quaternion;
  matrix.getRotation(quaternion);
  quaternion.normalize();
  return tf2::toMsg(quaternion);
}

cv::Vec3d rvec_from_rotation(const cv::Matx33d & rotation)
{
  cv::Mat rvec;
  cv::Rodrigues(cv::Mat(rotation), rvec);
  return {rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)};
}

}  // namespace

class D455PoseNode : public rclcpp::Node
{
public:
  D455PoseNode()
  : Node("d455_pose_node"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    mode_ = declare_parameter("mode", "bottle");
    enabled_ = declare_parameter("enabled", true);
    configuration_complete_ = declare_parameter("configuration_complete", true);
    color_topic_ = declare_parameter("color_topic", "/camera/camera/color/image_raw");
    depth_topic_ = declare_parameter(
      "aligned_depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    camera_info_topic_ = declare_parameter(
      "camera_info_topic", "/camera/camera/color/camera_info");
    output_frame_ = declare_parameter("output_frame", "camera_color_optical_frame");
    pose_topic_ = declare_parameter("pose_topic", "/d455_6d_pose/pose");
    detections_topic_ = declare_parameter(
      "detections_topic", "/d455_6d_pose/detections");
    annotated_topic_ = declare_parameter(
      "annotated_topic", "/d455_6d_pose/annotated");
    marker_topic_ = declare_parameter("marker_topic", "/d455_6d_pose/markers");
    diagnostic_topic_ = declare_parameter(
      "diagnostic_topic", "/d455_6d_pose/diagnostics");
    depth_scale_ = declare_parameter("depth_scale", 0.001);
    min_depth_m_ = declare_parameter("min_depth_m", 0.20);
    max_depth_m_ = declare_parameter("max_depth_m", 2.00);
    min_contour_area_px_ = declare_parameter("min_contour_area_px", 1200.0);
    min_confidence_ = declare_parameter("min_confidence", 0.45);
    max_sync_delta_s_ = declare_parameter("max_sync_delta_s", 0.08);
    smoothing_alpha_ = declare_parameter("smoothing_alpha", 0.35);
    reset_jump_m_ = declare_parameter("reset_jump_m", 0.20);
    hsv_.lower = cv::Scalar(
      declare_parameter("hsv_h_min", 90),
      declare_parameter("hsv_s_min", 60),
      declare_parameter("hsv_v_min", 40));
    hsv_.upper = cv::Scalar(
      declare_parameter("hsv_h_max", 135),
      declare_parameter("hsv_s_max", 255),
      declare_parameter("hsv_v_max", 255));
    bottle_min_aspect_ratio_ = declare_parameter("bottle_min_aspect_ratio", 1.35);
    bottle_diameter_m_ = declare_parameter("bottle_diameter_m", 0.070);
    bottle_height_m_ = declare_parameter("bottle_height_m", 0.230);
    energy_width_m_ = declare_parameter("energy_unit_width_m", 0.230);
    energy_height_m_ = declare_parameter("energy_unit_height_m", 0.127);
    energy_thickness_m_ = declare_parameter("energy_unit_thickness_m", 0.015);
    max_reprojection_error_px_ = declare_parameter("max_reprojection_error_px", 3.0);
    max_depth_residual_m_ = declare_parameter("max_depth_residual_m", 0.040);

    if (mode_ != "bottle" && mode_ != "energy_unit") {
      throw std::invalid_argument("mode must be 'bottle' or 'energy_unit'");
    }
    if (mode_ == "energy_unit" && !configuration_complete_) {
      RCLCPP_WARN(
        get_logger(),
        "Energy-unit configuration is intentionally incomplete; poses will be rejected until exact dimensions and target definition are approved");
    }

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 10);
    detection_publisher_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      detections_topic_, 10);
    annotated_publisher_ = create_publisher<Image>(annotated_topic_, 2);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, 10);
    diagnostic_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostic_topic_, 10);
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, rclcpp::SensorDataQoS(),
      std::bind(&D455PoseNode::on_camera_info, this, std::placeholders::_1));

    const auto qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
    color_subscription_.subscribe(this, color_topic_, qos);
    depth_subscription_.subscribe(this, depth_topic_, qos);
    synchronizer_ = std::make_shared<message_filters::Synchronizer<ApproximatePolicy>>(
      ApproximatePolicy(5), color_subscription_, depth_subscription_);
    synchronizer_->registerCallback(
      std::bind(
        &D455PoseNode::on_rgbd, this,
        std::placeholders::_1, std::placeholders::_2));

    enable_service_ = create_service<std_srvs::srv::SetBool>(
      "/d455_6d_pose/set_enabled",
      std::bind(
        &D455PoseNode::set_enabled, this,
        std::placeholders::_1, std::placeholders::_2));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/d455_6d_pose/reset_tracking",
      std::bind(
        &D455PoseNode::reset_tracking, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Independent D455 6D pose node configured: mode=%s, RGB=%s, depth=%s, output=%s",
      mode_.c_str(), color_topic_.c_str(), depth_topic_.c_str(), output_frame_.c_str());
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    CameraIntrinsics camera;
    camera.fx = message->k[0];
    camera.fy = message->k[4];
    camera.cx = message->k[2];
    camera.cy = message->k[5];
    camera.width = static_cast<int>(message->width);
    camera.height = static_cast<int>(message->height);
    if (!camera.valid()) {return;}
    std::lock_guard<std::mutex> lock(mutex_);
    camera_ = camera;
    camera_frame_ = message->header.frame_id;
  }

  void on_rgbd(const Image::ConstSharedPtr & color, const Image::ConstSharedPtr & depth)
  {
    if (!enabled_) {return;}
    CameraIntrinsics camera;
    std::string camera_frame;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!camera_) {
        publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::WARN,
          "waiting for CameraInfo", std::nullopt);
        return;
      }
      camera = *camera_;
      camera_frame = camera_frame_;
    }
    const auto color_time = rclcpp::Time(color->header.stamp).seconds();
    const auto depth_time = rclcpp::Time(depth->header.stamp).seconds();
    if (std::abs(color_time - depth_time) > max_sync_delta_s_) {
      publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "RGB/depth timestamp mismatch", std::nullopt);
      return;
    }
    if (color->width != depth->width || color->height != depth->height) {
      publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "depth is not aligned to color resolution", std::nullopt);
      return;
    }

    cv_bridge::CvImageConstPtr color_bridge;
    cv_bridge::CvImageConstPtr depth_bridge;
    try {
      color_bridge = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
      depth_bridge = cv_bridge::toCvShare(depth);
    } catch (const cv_bridge::Exception & error) {
      publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        std::string("cv_bridge: ") + error.what(), std::nullopt);
      return;
    }

    PoseEstimate estimate;
    if (mode_ == "bottle") {
      estimate = estimate_bottle_pose(
        color_bridge->image, depth_bridge->image, camera, hsv_, depth_scale_,
        min_depth_m_, max_depth_m_, min_contour_area_px_,
        bottle_min_aspect_ratio_);
    } else if (!configuration_complete_) {
      estimate.status = "energy-unit configuration incomplete";
    } else {
      estimate = estimate_planar_target_pose(
        color_bridge->image, depth_bridge->image, camera, hsv_,
        energy_width_m_, energy_height_m_, depth_scale_, min_depth_m_, max_depth_m_,
        min_contour_area_px_, max_reprojection_error_px_, max_depth_residual_m_);
    }

    cv::Mat annotated = color_bridge->image.clone();
    draw_annotation(annotated, estimate, camera);
    annotated_publisher_->publish(
      *cv_bridge::CvImage(color->header, "bgr8", annotated).toImageMsg());

    if (!estimate.valid || estimate.confidence < min_confidence_) {
      publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::WARN,
        estimate.status, estimate);
      return;
    }

    geometry_msgs::msg::PoseStamped camera_pose;
    camera_pose.header = color->header;
    if (!camera_frame.empty()) {camera_pose.header.frame_id = camera_frame;}
    camera_pose.pose.position.x = estimate.translation[0];
    camera_pose.pose.position.y = estimate.translation[1];
    camera_pose.pose.position.z = estimate.translation[2];
    camera_pose.pose.orientation = quaternion_from_rotation(estimate.rotation);
    auto output_pose = transform_pose(camera_pose);
    if (!output_pose) {
      publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "output TF unavailable", estimate);
      return;
    }
    output_pose->pose = smooth_pose(output_pose->pose);
    publish_pose(*output_pose, estimate);
    publish_detection(*output_pose, estimate);
    publish_marker(*output_pose, estimate);
    publish_diagnostic(color->header, diagnostic_msgs::msg::DiagnosticStatus::OK,
      estimate.status, estimate);
  }

  std::optional<geometry_msgs::msg::PoseStamped> transform_pose(
    const geometry_msgs::msg::PoseStamped & input)
  {
    if (output_frame_.empty() || output_frame_ == input.header.frame_id) {return input;}
    try {
      const auto transform = tf_buffer_.lookupTransform(
        output_frame_, input.header.frame_id, input.header.stamp,
        tf2::durationFromSec(0.10));
      geometry_msgs::msg::PoseStamped output;
      tf2::doTransform(input, output, transform);
      return output;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "%s", error.what());
      return std::nullopt;
    }
  }

  geometry_msgs::msg::Pose smooth_pose(const geometry_msgs::msg::Pose & input)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!smoothed_pose_) {
      smoothed_pose_ = input;
      return input;
    }
    const auto distance = std::sqrt(
      std::pow(input.position.x - smoothed_pose_->position.x, 2) +
      std::pow(input.position.y - smoothed_pose_->position.y, 2) +
      std::pow(input.position.z - smoothed_pose_->position.z, 2));
    if (distance > reset_jump_m_) {
      smoothed_pose_ = input;
      return input;
    }
    const auto alpha = std::clamp(smoothing_alpha_, 0.0, 1.0);
    smoothed_pose_->position.x += alpha * (input.position.x - smoothed_pose_->position.x);
    smoothed_pose_->position.y += alpha * (input.position.y - smoothed_pose_->position.y);
    smoothed_pose_->position.z += alpha * (input.position.z - smoothed_pose_->position.z);
    tf2::Quaternion previous;
    tf2::Quaternion current;
    tf2::fromMsg(smoothed_pose_->orientation, previous);
    tf2::fromMsg(input.orientation, current);
    if (previous.dot(current) < 0.0) {current = tf2::Quaternion(-current.x(), -current.y(), -current.z(), -current.w());}
    auto filtered = previous.slerp(current, alpha);
    filtered.normalize();
    smoothed_pose_->orientation = tf2::toMsg(filtered);
    return *smoothed_pose_;
  }

  void publish_pose(
    const geometry_msgs::msg::PoseStamped & pose, const PoseEstimate & estimate)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header = pose.header;
    message.pose.pose = pose.pose;
    message.pose.covariance.fill(0.0);
    const auto translation_variance = std::pow(
      std::max(0.003, std::isfinite(estimate.depth_residual_m) ?
      estimate.depth_residual_m : 0.015), 2);
    message.pose.covariance[0] = translation_variance;
    message.pose.covariance[7] = translation_variance;
    message.pose.covariance[14] = translation_variance * 2.0;
    const auto measured_rotation_variance = std::pow(
      std::max(1.0, estimate.reprojection_error_px) * CV_PI / 180.0, 2);
    const auto rotation_variance = estimate.orientation_complete ?
      measured_rotation_variance : 1.0e3;
    // Axial ambiguity is expressed in the object frame while ROS covariance
    // is fixed-axis. Mark all rotational axes unknown rather than putting the
    // uncertainty into an arbitrary camera-frame yaw component.
    message.pose.covariance[21] = rotation_variance;
    message.pose.covariance[28] = rotation_variance;
    message.pose.covariance[35] = rotation_variance;
    pose_publisher_->publish(message);
  }

  void publish_detection(
    const geometry_msgs::msg::PoseStamped & pose, const PoseEstimate & estimate)
  {
    vision_msgs::msg::Detection3DArray array;
    array.header = pose.header;
    vision_msgs::msg::Detection3D detection;
    detection.header = pose.header;
    detection.id = mode_ + "_0001";
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = mode_;
    hypothesis.hypothesis.score = estimate.confidence;
    hypothesis.pose.pose = pose.pose;
    detection.results.push_back(hypothesis);
    detection.bbox.center = pose.pose;
    if (mode_ == "bottle") {
      detection.bbox.size.x = bottle_diameter_m_;
      detection.bbox.size.y = bottle_diameter_m_;
      detection.bbox.size.z = bottle_height_m_;
    } else {
      detection.bbox.size.x = energy_width_m_;
      detection.bbox.size.y = energy_height_m_;
      detection.bbox.size.z = energy_thickness_m_;
    }
    array.detections.push_back(detection);
    detection_publisher_->publish(array);
  }

  void publish_marker(
    const geometry_msgs::msg::PoseStamped & pose, const PoseEstimate & estimate)
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = "d455_6d_pose";
    marker.id = 0;
    marker.type = mode_ == "bottle" ?
      visualization_msgs::msg::Marker::CYLINDER :
      visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    if (mode_ == "bottle") {
      marker.scale.x = bottle_diameter_m_;
      marker.scale.y = bottle_diameter_m_;
      marker.scale.z = bottle_height_m_;
    } else {
      marker.scale.x = energy_width_m_;
      marker.scale.y = energy_height_m_;
      marker.scale.z = energy_thickness_m_;
    }
    marker.color.r = estimate.orientation_complete ? 0.1F : 1.0F;
    marker.color.g = estimate.orientation_complete ? 0.9F : 0.7F;
    marker.color.b = 0.1F;
    marker.color.a = 0.55F;
    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    array.markers.push_back(marker);
    marker_publisher_->publish(array);
  }

  void publish_diagnostic(
    const std_msgs::msg::Header & header, std::uint8_t level,
    const std::string & message, const std::optional<PoseEstimate> & estimate)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header = header;
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.level = level;
    status.name = "d455_6d_pose/estimator";
    status.hardware_id = "intel_realsense_d455";
    status.message = message;
    status.values.push_back(key_value("mode", mode_));
    status.values.push_back(key_value("output_frame", output_frame_));
    if (estimate) {
      status.values.push_back(key_value("confidence", number(estimate->confidence)));
      status.values.push_back(key_value("depth_valid_ratio", number(estimate->depth_valid_ratio)));
      status.values.push_back(key_value("reprojection_error_px", number(estimate->reprojection_error_px)));
      status.values.push_back(key_value("depth_residual_m", number(estimate->depth_residual_m)));
      status.values.push_back(key_value(
        "orientation_complete", estimate->orientation_complete ? "true" : "false"));
    }
    array.status.push_back(status);
    diagnostic_publisher_->publish(array);
  }

  void draw_annotation(
    cv::Mat & image, const PoseEstimate & estimate, const CameraIntrinsics & camera)
  {
    const auto color = estimate.valid ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
    if (estimate.bounding_box.area() > 0) {cv::rectangle(image, estimate.bounding_box, color, 2);}
    if (mode_ == "energy_unit") {
      for (std::size_t index = 0; index < estimate.image_corners.size(); ++index) {
        cv::circle(image, estimate.image_corners[index], 5, cv::Scalar(0, 255, 255), -1);
        cv::putText(image, std::to_string(index), estimate.image_corners[index],
          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
      }
    }
    if (estimate.valid && estimate.orientation_complete) {
      const auto rvec = rvec_from_rotation(estimate.rotation);
      cv::drawFrameAxes(image, camera.matrix(), cv::Mat(), rvec, estimate.translation, 0.08F);
    }
    cv::putText(image, mode_ + ": " + estimate.status, {15, 30},
      cv::FONT_HERSHEY_SIMPLEX, 0.60, color, 2, cv::LINE_AA);
    cv::putText(image, "score=" + number(estimate.confidence, 2), {15, 58},
      cv::FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv::LINE_AA);
  }

  void set_enabled(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    enabled_ = request->data;
    if (!enabled_) {
      std::lock_guard<std::mutex> lock(mutex_);
      smoothed_pose_.reset();
    }
    response->success = true;
    response->message = enabled_ ? "6D estimator enabled" : "6D estimator disabled";
  }

  void reset_tracking(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    smoothed_pose_.reset();
    response->success = true;
    response->message = "pose smoothing state reset";
  }

  std::mutex mutex_;
  std::optional<CameraIntrinsics> camera_;
  std::string camera_frame_;
  std::optional<geometry_msgs::msg::Pose> smoothed_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::string mode_;
  bool enabled_;
  bool configuration_complete_;
  std::string color_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string output_frame_;
  std::string pose_topic_;
  std::string detections_topic_;
  std::string annotated_topic_;
  std::string marker_topic_;
  std::string diagnostic_topic_;
  HsvRange hsv_;
  double depth_scale_;
  double min_depth_m_;
  double max_depth_m_;
  double min_contour_area_px_;
  double min_confidence_;
  double max_sync_delta_s_;
  double smoothing_alpha_;
  double reset_jump_m_;
  double bottle_min_aspect_ratio_;
  double bottle_diameter_m_;
  double bottle_height_m_;
  double energy_width_m_;
  double energy_height_m_;
  double energy_thickness_m_;
  double max_reprojection_error_px_;
  double max_depth_residual_m_;

  message_filters::Subscriber<Image> color_subscription_;
  message_filters::Subscriber<Image> depth_subscription_;
  std::shared_ptr<message_filters::Synchronizer<ApproximatePolicy>> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_publisher_;
  rclcpp::Publisher<Image>::SharedPtr annotated_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_publisher_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

}  // namespace d455_6d_pose

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<d455_6d_pose::D455PoseNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("d455_pose_node"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
