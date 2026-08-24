#include <algorithm>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace fruit_picking_arm
{

/// Gazebo-only grasp constraint for the simulated parallel-jaw gripper.
///
/// The fingers first close to the measured fruit radius and make physical
/// contact.  A fixed Gazebo joint is then created to model static friction and
/// normal force without requiring unrealistically stiff contact gains.  On
/// release the joint is removed and gravity/contact physics take over.
class FruitGraspAttachPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = std::move(model);
    world_ = model_->GetWorld();
    ros_node_ = gazebo_ros::Node::Get(sdf);

    parent_link_name_ =
      sdf->Get<std::string>("parent_link", "gripper_base").first;
    child_link_name_ =
      sdf->Get<std::string>("child_link", "apple_link").first;
    model_prefix_ = sdf->Get<std::string>("model_prefix", "fruit_").first;
    max_distance_ = sdf->Get<double>("max_distance", 0.13).first;

    parent_link_ = model_->GetLink(parent_link_name_);
    if (!parent_link_) {
      RCLCPP_ERROR(
        ros_node_->get_logger(), "Fruit gripper parent link '%s' not found",
        parent_link_name_.c_str());
      return;
    }

    service_ = ros_node_->create_service<std_srvs::srv::SetBool>(
      "set_attached",
      std::bind(
        &FruitGraspAttachPlugin::OnSetAttached, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
      ros_node_->get_logger(),
      "Physical fruit grasp service ready (parent=%s, max_distance=%.3fm)",
      parent_link_name_.c_str(), max_distance_);
  }

  ~FruitGraspAttachPlugin() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Detach();
  }

private:
  void OnSetAttached(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!parent_link_) {
      response->success = false;
      response->message = "parent gripper link is unavailable";
      return;
    }

    if (!request->data) {
      if (!fixed_joint_) {
        response->success = true;
        response->message = "no fruit was attached";
        return;
      }
      const auto released = attached_model_name_;
      Detach();
      response->success = true;
      response->message = "detached " + released;
      RCLCPP_INFO(
        ros_node_->get_logger(), "PHYSICAL_GRASP_DETACHED model=%s",
        released.c_str());
      return;
    }

    if (fixed_joint_) {
      response->success = true;
      response->message = "already attached " + attached_model_name_;
      return;
    }

    gazebo::physics::ModelPtr nearest_model;
    gazebo::physics::LinkPtr nearest_link;
    double nearest_distance = max_distance_;
    const auto gripper_position = parent_link_->WorldPose().Pos();

    for (const auto & candidate : world_->Models()) {
      if (!candidate ||
        candidate->GetName().compare(0, model_prefix_.size(), model_prefix_) != 0)
      {
        continue;
      }
      const auto link = candidate->GetLink(child_link_name_);
      if (!link) {
        continue;
      }
      const double distance =
        gripper_position.Distance(link->WorldPose().Pos());
      if (distance <= nearest_distance) {
        nearest_distance = distance;
        nearest_model = candidate;
        nearest_link = link;
      }
    }

    if (!nearest_model || !nearest_link) {
      response->success = false;
      response->message = "no fruit is within grasp distance";
      RCLCPP_WARN(
        ros_node_->get_logger(),
        "Physical grasp rejected: no fruit within %.3fm", max_distance_);
      return;
    }

    fixed_joint_ = model_->CreateJoint(
      "fruit_grasp_fixed_joint", "fixed", parent_link_, nearest_link);
    if (!fixed_joint_) {
      response->success = false;
      response->message = "Gazebo failed to create fixed grasp joint";
      return;
    }
    fixed_joint_->Init();
    attached_model_name_ = nearest_model->GetName();

    response->success = true;
    response->message = "attached " + attached_model_name_;
    RCLCPP_INFO(
      ros_node_->get_logger(),
      "PHYSICAL_GRASP_ATTACHED model=%s distance=%.3fm",
      attached_model_name_.c_str(), nearest_distance);
  }

  void Detach()
  {
    if (!fixed_joint_) {
      return;
    }
    fixed_joint_->Detach();
    model_->RemoveJoint("fruit_grasp_fixed_joint");
    fixed_joint_.reset();
    attached_model_name_.clear();
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr parent_link_;
  gazebo::physics::JointPtr fixed_joint_;
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
  std::mutex mutex_;
  std::string parent_link_name_;
  std::string child_link_name_;
  std::string model_prefix_;
  std::string attached_model_name_;
  double max_distance_{0.13};
};

GZ_REGISTER_MODEL_PLUGIN(FruitGraspAttachPlugin)

}  // namespace fruit_picking_arm
