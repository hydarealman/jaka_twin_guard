#include "fruit_picking_arm/eye_to_hand_solver.hpp"

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fruit_picking_arm
{
namespace calibration
{
namespace
{

using Trigger = std_srvs::srv::Trigger;

struct Detection
{
  Transform3d camera_T_target;
  rclcpp::Time stamp;
  std::string frame_id;
  int corner_count{0};
  double reprojection_error_px{0.0};
};

cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_from_name(const std::string & name)
{
  if (name == "DICT_4X4_50") {return cv::aruco::DICT_4X4_50;}
  if (name == "DICT_4X4_100") {return cv::aruco::DICT_4X4_100;}
  if (name == "DICT_5X5_50") {return cv::aruco::DICT_5X5_50;}
  if (name == "DICT_5X5_100") {return cv::aruco::DICT_5X5_100;}
  if (name == "DICT_5X5_250") {return cv::aruco::DICT_5X5_250;}
  if (name == "DICT_6X6_250") {return cv::aruco::DICT_6X6_250;}
  throw std::invalid_argument("unsupported ArUco dictionary: " + name);
}

Transform3d transform_from_message(const geometry_msgs::msg::Transform & message)
{
  tf2::Quaternion quaternion(
    message.rotation.x, message.rotation.y,
    message.rotation.z, message.rotation.w);
  if (quaternion.length2() < 1e-12) {
    throw std::runtime_error("TF contains an invalid zero quaternion");
  }
  quaternion.normalize();
  tf2::Matrix3x3 matrix(quaternion);
  Transform3d value;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      value.rotation(row, column) = matrix[row][column];
    }
  }
  value.translation = {
    message.translation.x, message.translation.y, message.translation.z};
  return value;
}

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

Transform3d pose_from_rvec_tvec(const cv::Vec3d & rvec, const cv::Vec3d & tvec)
{
  cv::Mat rotation;
  cv::Rodrigues(rvec, rotation);
  Transform3d value;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      value.rotation(row, column) = rotation.at<double>(row, column);
    }
  }
  value.translation = tvec;
  return value;
}

std::array<double, 4> quaternion_xyzw(const cv::Matx33d & rotation)
{
  tf2::Matrix3x3 matrix(
    rotation(0, 0), rotation(0, 1), rotation(0, 2),
    rotation(1, 0), rotation(1, 1), rotation(1, 2),
    rotation(2, 0), rotation(2, 1), rotation(2, 2));
  tf2::Quaternion quaternion;
  matrix.getRotation(quaternion);
  quaternion.normalize();
  return {quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w()};
}

double reprojection_error(
  const std::vector<cv::Point3f> & object_points,
  const std::vector<cv::Point2f> & image_points,
  const cv::Vec3d & rvec, const cv::Vec3d & tvec,
  const cv::Mat & camera_matrix, const cv::Mat & distortion)
{
  std::vector<cv::Point2f> projected;
  cv::projectPoints(
    object_points, rvec, tvec, camera_matrix, distortion, projected);
  if (projected.empty()) {return 0.0;}
  double squared = 0.0;
  for (std::size_t index = 0; index < projected.size(); ++index) {
    const auto delta = projected[index] - image_points[index];
    squared += delta.dot(delta);
  }
  return std::sqrt(squared / static_cast<double>(projected.size()));
}

std::string vector_yaml(const cv::Vec3d & value)
{
  std::ostringstream stream;
  stream << std::scientific << std::setprecision(12) << "[" << value[0] << ", " << value[1]
         << ", " << value[2] << "]";
  return stream.str();
}

}  // namespace

class EyeToHandCalibrator : public rclcpp::Node
{
public:
  EyeToHandCalibrator()
  : Node("eye_to_hand_calibrator"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    image_topic_ = declare_parameter("image_topic", "/camera/camera/color/image_raw");
    camera_info_topic_ = declare_parameter(
      "camera_info_topic", "/camera/camera/color/camera_info");
    annotated_topic_ = declare_parameter(
      "annotated_topic", "/hand_eye_calibration/annotated");
    base_frame_ = declare_parameter("base_frame", "base_link");
    tool_frame_ = declare_parameter("tool_frame", "tool_flange");
    camera_frame_ = declare_parameter("camera_frame", "camera_color_optical_frame");
    camera_root_frame_ = declare_parameter("camera_root_frame", "camera_link");
    target_type_ = declare_parameter("target_type", "charuco");
    output_yaml_ = declare_parameter("output_yaml", "hand_eye_params.yaml");
    dictionary_name_ = declare_parameter("dictionary", "DICT_5X5_250");
    charuco_squares_x_ = declare_parameter("charuco_squares_x", 7);
    charuco_squares_y_ = declare_parameter("charuco_squares_y", 5);
    square_length_m_ = declare_parameter("square_length_m", 0.030);
    marker_length_m_ = declare_parameter("marker_length_m", 0.022);
    chessboard_corners_x_ = declare_parameter("chessboard_corners_x", 9);
    chessboard_corners_y_ = declare_parameter("chessboard_corners_y", 6);
    min_detected_corners_ = declare_parameter("min_detected_corners", 8);
    min_samples_ = declare_parameter("min_samples", 12);
    max_detection_age_s_ = declare_parameter("max_detection_age_s", 0.75);
    duplicate_translation_m_ = declare_parameter("duplicate_translation_m", 0.010);
    duplicate_rotation_deg_ = declare_parameter("duplicate_rotation_deg", 3.0);
    max_reprojection_error_px_ = declare_parameter("max_reprojection_error_px", 1.5);
    max_translation_rms_m_ = declare_parameter("max_translation_rms_m", 0.005);
    max_rotation_rms_deg_ = declare_parameter("max_rotation_rms_deg", 1.0);
    allow_poor_quality_save_ = declare_parameter("allow_poor_quality_save", false);

    if (target_type_ != "charuco" && target_type_ != "chessboard") {
      throw std::invalid_argument("target_type must be 'charuco' or 'chessboard'");
    }
    if (square_length_m_ <= 0.0 || marker_length_m_ <= 0.0 ||
      marker_length_m_ >= square_length_m_)
    {
      throw std::invalid_argument(
              "board dimensions must satisfy 0 < marker_length_m < square_length_m");
    }
    dictionary_ = cv::aruco::getPredefinedDictionary(
      dictionary_from_name(dictionary_name_));
    charuco_board_ = cv::aruco::CharucoBoard::create(
      charuco_squares_x_, charuco_squares_y_,
      static_cast<float>(square_length_m_),
      static_cast<float>(marker_length_m_), dictionary_);

    const auto image_qos = rclcpp::SensorDataQoS();
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, image_qos,
      std::bind(&EyeToHandCalibrator::on_image, this, std::placeholders::_1));
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, image_qos,
      std::bind(&EyeToHandCalibrator::on_camera_info, this, std::placeholders::_1));
    annotated_publisher_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, 2);

    capture_service_ = create_service<Trigger>(
      "/hand_eye_calibration/capture",
      std::bind(
        &EyeToHandCalibrator::capture, this,
        std::placeholders::_1, std::placeholders::_2));
    remove_last_service_ = create_service<Trigger>(
      "/hand_eye_calibration/remove_last",
      std::bind(
        &EyeToHandCalibrator::remove_last, this,
        std::placeholders::_1, std::placeholders::_2));
    clear_service_ = create_service<Trigger>(
      "/hand_eye_calibration/clear",
      std::bind(
        &EyeToHandCalibrator::clear, this,
        std::placeholders::_1, std::placeholders::_2));
    solve_service_ = create_service<Trigger>(
      "/hand_eye_calibration/solve",
      std::bind(
        &EyeToHandCalibrator::solve, this,
        std::placeholders::_1, std::placeholders::_2));
    save_service_ = create_service<Trigger>(
      "/hand_eye_calibration/save",
      std::bind(
        &EyeToHandCalibrator::save, this,
        std::placeholders::_1, std::placeholders::_2));
    status_service_ = create_service<Trigger>(
      "/hand_eye_calibration/status",
      std::bind(
        &EyeToHandCalibrator::status, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "C++ eye-to-hand calibrator ready: target=%s, image=%s, %s -> %s",
      target_type_.c_str(), image_topic_.c_str(), base_frame_.c_str(),
      camera_frame_.c_str());
    RCLCPP_WARN(
      get_logger(),
      "Board dimensions are metric calibration inputs; verify the printed/glass board with a ruler before sampling");
  }

private:
  void on_camera_info(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    if (message->k[0] <= 0.0 || message->k[4] <= 0.0) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Ignoring invalid CameraInfo intrinsics");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    camera_info_ = *message;
  }

  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr message)
  {
    sensor_msgs::msg::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!camera_info_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Waiting for CameraInfo on %s", camera_info_topic_.c_str());
        return;
      }
      camera_info = *camera_info_;
    }

    cv_bridge::CvImageConstPtr bridge;
    try {
      bridge = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "cv_bridge failed: %s", error.what());
      return;
    }
    cv::Mat annotated = bridge->image.clone();
    std::optional<Detection> detection;
    try {
      if (target_type_ == "charuco") {
        detection = detect_charuco(annotated, camera_info, message->header);
      } else {
        detection = detect_chessboard(annotated, camera_info, message->header);
      }
    } catch (const cv::Exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000, "Target detection failed: %s", error.what());
    }

    if (detection) {
      const bool acceptable =
        detection->corner_count >= min_detected_corners_ &&
        detection->reprojection_error_px <= max_reprojection_error_px_;
      const cv::Scalar color = acceptable ? cv::Scalar(0, 200, 0) : cv::Scalar(0, 0, 255);
      std::ostringstream label;
      label << (acceptable ? "READY" : "REJECT") << " corners="
            << detection->corner_count << " reproj=" << std::fixed
            << std::setprecision(2) << detection->reprojection_error_px << " px";
      cv::putText(
        annotated, label.str(), cv::Point(20, 35), cv::FONT_HERSHEY_SIMPLEX,
        0.75, color, 2, cv::LINE_AA);
      std::lock_guard<std::mutex> lock(mutex_);
      latest_detection_ = acceptable ? detection : std::nullopt;
    } else {
      cv::putText(
        annotated, "NO CALIBRATION TARGET", cv::Point(20, 35),
        cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
      std::lock_guard<std::mutex> lock(mutex_);
      latest_detection_.reset();
    }
    annotated_publisher_->publish(
      *cv_bridge::CvImage(message->header, "bgr8", annotated).toImageMsg());
  }

  std::optional<Detection> detect_charuco(
    cv::Mat & image, const sensor_msgs::msg::CameraInfo & info,
    const std_msgs::msg::Header & header)
  {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids);
    if (marker_ids.empty()) {return std::nullopt;}
    cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

    std::vector<cv::Point2f> corners;
    std::vector<int> ids;
    const cv::Mat camera_matrix = camera_matrix_from(info);
    const cv::Mat distortion = distortion_from(info);
    cv::aruco::interpolateCornersCharuco(
      marker_corners, marker_ids, image, charuco_board_, corners, ids,
      camera_matrix, distortion);
    if (ids.size() < 4) {return std::nullopt;}
    cv::aruco::drawDetectedCornersCharuco(image, corners, ids);

    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if (!cv::aruco::estimatePoseCharucoBoard(
        corners, ids, charuco_board_, camera_matrix, distortion,
        rvec, tvec, false))
    {
      return std::nullopt;
    }
    cv::drawFrameAxes(
      image, camera_matrix, distortion, rvec, tvec,
      static_cast<float>(square_length_m_ * 2.0));
    std::vector<cv::Point3f> object_points;
    object_points.reserve(ids.size());
    for (const auto id : ids) {
      object_points.push_back(charuco_board_->chessboardCorners.at(id));
    }
    return Detection{
      pose_from_rvec_tvec(rvec, tvec), rclcpp::Time(header.stamp),
      header.frame_id, static_cast<int>(ids.size()),
      reprojection_error(
        object_points, corners, rvec, tvec, camera_matrix, distortion)};
  }

  std::optional<Detection> detect_chessboard(
    cv::Mat & image, const sensor_msgs::msg::CameraInfo & info,
    const std_msgs::msg::Header & header)
  {
    const cv::Size pattern(chessboard_corners_x_, chessboard_corners_y_);
    std::vector<cv::Point2f> corners;
    if (!cv::findChessboardCorners(
        image, pattern, corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE))
    {
      return std::nullopt;
    }
    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(
      gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(
        cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));
    cv::drawChessboardCorners(image, pattern, corners, true);

    std::vector<cv::Point3f> object_points;
    object_points.reserve(corners.size());
    for (int row = 0; row < chessboard_corners_y_; ++row) {
      for (int column = 0; column < chessboard_corners_x_; ++column) {
        object_points.emplace_back(
          static_cast<float>(column * square_length_m_),
          static_cast<float>(row * square_length_m_), 0.0F);
      }
    }
    const cv::Mat camera_matrix = camera_matrix_from(info);
    const cv::Mat distortion = distortion_from(info);
    cv::Vec3d rvec;
    cv::Vec3d tvec;
    if (!cv::solvePnP(
        object_points, corners, camera_matrix, distortion,
        rvec, tvec, false, cv::SOLVEPNP_ITERATIVE))
    {
      return std::nullopt;
    }
    cv::drawFrameAxes(
      image, camera_matrix, distortion, rvec, tvec,
      static_cast<float>(square_length_m_ * 2.0));
    return Detection{
      pose_from_rvec_tvec(rvec, tvec), rclcpp::Time(header.stamp),
      header.frame_id, static_cast<int>(corners.size()),
      reprojection_error(
        object_points, corners, rvec, tvec, camera_matrix, distortion)};
  }

  void capture(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::optional<Detection> detection;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      detection = latest_detection_;
    }
    if (!detection) {
      fail(response, "no valid board pose; check the annotated image");
      return;
    }
    const auto age = (now() - detection->stamp).seconds();
    if (age < -0.1 || age > max_detection_age_s_) {
      fail(response, "latest board detection is stale");
      return;
    }
    if (!detection->frame_id.empty() && detection->frame_id != camera_frame_) {
      fail(
        response, "image frame is '" + detection->frame_id +
        "' but camera_frame is '" + camera_frame_ + "'");
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(
        base_frame_, tool_frame_, tf2::TimePointZero,
        tf2::durationFromSec(0.25));
    } catch (const tf2::TransformException & error) {
      fail(response, "cannot read base-to-tool TF: " + std::string(error.what()));
      return;
    }
    EyeToHandSample sample{
      transform_from_message(transform.transform), detection->camera_T_target};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!samples_.empty()) {
        const auto & previous = samples_.back().base_T_tool;
        const auto translation_delta = cv::norm(
          sample.base_T_tool.translation - previous.translation);
        const auto rotation_delta = rotation_distance_deg(
          sample.base_T_tool.rotation, previous.rotation);
        if (translation_delta < duplicate_translation_m_ &&
          rotation_delta < duplicate_rotation_deg_)
        {
          fail(
            response, "pose is too similar to the previous sample; move or rotate the tool");
          return;
        }
      }
      samples_.push_back(sample);
      sample_reprojection_errors_.push_back(detection->reprojection_error_px);
      result_.reset();
      response->success = true;
      response->message = "captured sample " + std::to_string(samples_.size()) +
        ", reprojection=" + std::to_string(detection->reprojection_error_px) + " px";
    }
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  void remove_last(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) {
      fail(response, "there are no samples to remove");
      return;
    }
    samples_.pop_back();
    sample_reprojection_errors_.pop_back();
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
    sample_reprojection_errors_.clear();
    result_.reset();
    response->success = true;
    response->message = "all calibration samples cleared";
  }

  void solve(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::vector<EyeToHandSample> samples;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      samples = samples_;
    }
    try {
      auto result = solve_eye_to_hand(samples, static_cast<std::size_t>(min_samples_));
      const bool good = quality_is_good(result);
      std::ostringstream message;
      message << "method=" << result.method << ", samples=" << samples.size()
              << ", translation_rms=" << std::fixed << std::setprecision(3)
              << result.metrics.translation_rms_m * 1000.0 << " mm"
              << ", rotation_rms=" << result.metrics.rotation_rms_deg << " deg"
              << ", rotation_span=" << result.metrics.robot_rotation_span_deg << " deg"
              << (good ? ", quality=PASS" : ", quality=FAIL");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        result_ = result;
      }
      response->success = good;
      response->message = message.str();
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      fail(response, error.what());
    }
  }

  void save(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::optional<EyeToHandResult> result;
    std::vector<EyeToHandSample> samples;
    std::vector<double> reprojection_errors;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      result = result_;
      samples = samples_;
      reprojection_errors = sample_reprojection_errors_;
    }
    if (!result) {
      fail(response, "no solution available; call /hand_eye_calibration/solve first");
      return;
    }
    if (!quality_is_good(*result) && !allow_poor_quality_save_) {
      fail(response, "solution failed quality limits; collect better poses before saving");
      return;
    }
    try {
      EyeToHandResult published_result = *result;
      if (camera_root_frame_ != camera_frame_) {
        // The solver observes the optical frame, while the RealSense driver
        // already owns camera_link -> optical-frame TFs.  Publish only
        // base_link -> camera_link so the optical frame never gets two TF
        // parents.
        const auto root_to_optical = tf_buffer_.lookupTransform(
          camera_root_frame_, camera_frame_, tf2::TimePointZero);
        published_result.base_T_camera = compose(
          result->base_T_camera,
          inverse(transform_from_message(root_to_optical.transform)));
      }
      write_result(published_result);
      write_samples(*result, samples, reprojection_errors);
      response->success = true;
      response->message = "saved " + base_frame_ + "-to-" + camera_root_frame_ +
        " transform to " + output_yaml_;
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      fail(response, "save failed: " + std::string(error.what()));
    }
  }

  void status(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    response->success = true;
    response->message = "samples=" + std::to_string(samples_.size()) +
      "/" + std::to_string(min_samples_) +
      ", detection=" + (latest_detection_ ? "ready" : "not_ready") +
      ", solution=" + (result_ ? "available" : "none");
  }

  bool quality_is_good(const EyeToHandResult & result) const
  {
    return result.metrics.translation_rms_m <= max_translation_rms_m_ &&
           result.metrics.rotation_rms_deg <= max_rotation_rms_deg_;
  }

  void write_result(const EyeToHandResult & result) const
  {
    const auto destination = output_destination();
    if (!destination.parent_path().empty()) {
      std::filesystem::create_directories(destination.parent_path());
    }
    const auto temporary = destination.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) {throw std::runtime_error("cannot open temporary output file");}
    const auto quaternion = quaternion_xyzw(result.base_T_camera.rotation);
    output << std::scientific << std::setprecision(12)
           << "# Generated by eye_to_hand_calibrator; transform convention: parent_T_child\n"
           << "# Method: " << result.method << ", translation RMS: "
           << result.metrics.translation_rms_m << " m, rotation RMS: "
           << result.metrics.rotation_rms_deg << " deg\n"
           << "hand_eye_static_tf:\n"
           << "  ros__parameters:\n"
           << "    calibrated: true\n"
           << "    parent_frame: \"" << base_frame_ << "\"\n"
           << "    child_frame: \"" << camera_root_frame_ << "\"\n"
           << "    translation_m: " << vector_yaml(result.base_T_camera.translation) << "\n"
           << "    quaternion_xyzw: [" << quaternion[0] << ", " << quaternion[1]
           << ", " << quaternion[2] << ", " << quaternion[3] << "]\n";
    output.close();
    if (!output) {throw std::runtime_error("failed while writing output file");}
    if (std::filesystem::exists(destination)) {
      std::filesystem::copy_file(
        destination, destination.string() + ".bak",
        std::filesystem::copy_options::overwrite_existing);
      std::filesystem::remove(destination);
    }
    std::filesystem::rename(temporary, destination);
  }

  void write_samples(
    const EyeToHandResult & result,
    const std::vector<EyeToHandSample> & samples,
    const std::vector<double> & reprojection_errors) const
  {
    cv::FileStorage storage(
      output_destination().string() + ".samples.yaml", cv::FileStorage::WRITE);
    if (!storage.isOpened()) {throw std::runtime_error("cannot open sample output file");}
    storage << "base_frame" << base_frame_;
    storage << "tool_frame" << tool_frame_;
    storage << "camera_frame" << camera_frame_;
    storage << "camera_root_frame" << camera_root_frame_;
    storage << "solver_method" << result.method;
    storage << "translation_rms_m" << result.metrics.translation_rms_m;
    storage << "rotation_rms_deg" << result.metrics.rotation_rms_deg;
    storage << "samples" << "[";
    for (std::size_t index = 0; index < samples.size(); ++index) {
      storage << "{";
      storage << "base_R_tool" << cv::Mat(samples[index].base_T_tool.rotation);
      storage << "base_t_tool" << cv::Mat(samples[index].base_T_tool.translation);
      storage << "camera_R_target" << cv::Mat(samples[index].camera_T_target.rotation);
      storage << "camera_t_target" << cv::Mat(samples[index].camera_T_target.translation);
      storage << "reprojection_error_px" << reprojection_errors[index];
      storage << "}";
    }
    storage << "]";
  }

  std::filesystem::path output_destination() const
  {
    const std::filesystem::path configured(output_yaml_);
    // colcon --symlink-install makes config files symbolic links. Resolve the
    // link so saving updates the source configuration instead of replacing
    // only the generated install-space link.
    if (std::filesystem::is_symlink(configured)) {
      return std::filesystem::canonical(configured);
    }
    return configured;
  }

  static void fail(
    const std::shared_ptr<Trigger::Response> & response,
    const std::string & message)
  {
    response->success = false;
    response->message = message;
  }

  std::mutex mutex_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<sensor_msgs::msg::CameraInfo> camera_info_;
  std::optional<Detection> latest_detection_;
  std::vector<EyeToHandSample> samples_;
  std::vector<double> sample_reprojection_errors_;
  std::optional<EyeToHandResult> result_;

  std::string image_topic_;
  std::string camera_info_topic_;
  std::string annotated_topic_;
  std::string base_frame_;
  std::string tool_frame_;
  std::string camera_frame_;
  std::string camera_root_frame_;
  std::string target_type_;
  std::string output_yaml_;
  std::string dictionary_name_;
  int charuco_squares_x_;
  int charuco_squares_y_;
  int chessboard_corners_x_;
  int chessboard_corners_y_;
  int min_detected_corners_;
  int min_samples_;
  double square_length_m_;
  double marker_length_m_;
  double max_detection_age_s_;
  double duplicate_translation_m_;
  double duplicate_rotation_deg_;
  double max_reprojection_error_px_;
  double max_translation_rms_m_;
  double max_rotation_rms_deg_;
  bool allow_poor_quality_save_;

  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_publisher_;
  rclcpp::Service<Trigger>::SharedPtr capture_service_;
  rclcpp::Service<Trigger>::SharedPtr remove_last_service_;
  rclcpp::Service<Trigger>::SharedPtr clear_service_;
  rclcpp::Service<Trigger>::SharedPtr solve_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr status_service_;
};

}  // namespace calibration
}  // namespace fruit_picking_arm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<fruit_picking_arm::calibration::EyeToHandCalibrator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("eye_to_hand_calibrator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
