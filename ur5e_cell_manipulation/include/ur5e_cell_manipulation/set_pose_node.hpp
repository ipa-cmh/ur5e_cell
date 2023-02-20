#ifndef SET_POSE_NODE_HPP
#define SET_POSE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.hpp>

class SetPoseNode : public BT::SyncActionNode
{
protected:
  rclcpp::Node::SharedPtr node_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  std::string planning_group_;
  bool first_;

public:
  SetPoseNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialising Plan Node.");
    moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
    planning_component_ =
        config.blackboard->get<moveit_cpp::PlanningComponentPtr>("planning_component");
    planning_group_ = config.blackboard->get<std::string>("planning_group");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("positionx"),
             BT::InputPort<double>("positiony"),
             BT::InputPort<double>("positionz"),
             BT::InputPort<double>("roll"),
             BT::InputPort<double>("pitch"),
             BT::InputPort<double>("yaw"),
             BT::InputPort<std::string>("planning_frame"),
             BT::InputPort<std::string>("moving_link"),
             BT::OutputPort<moveit::core::RobotStatePtr>("robot_state") };
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "Setting a pose.");
    if (!(getInput<std::string>("planning_frame").has_value() &&
          getInput<std::string>("moving_link").has_value() &&
          getInput<double>("positionx").has_value() && getInput<double>("positiony").has_value() &&
          getInput<double>("positionz").has_value() && getInput<double>("roll").has_value() &&
          getInput<double>("pitch").has_value() && getInput<double>("yaw").has_value()))
    {
      RCLCPP_ERROR(node_->get_logger(), "Missing input parameters.");
      return BT::NodeStatus::FAILURE;
    }

    auto planning_frame = getInput<std::string>("planning_frame").value();
    auto moving_link = getInput<std::string>("moving_link").value();
    auto positionx = getInput<double>("positionx").value();
    auto positiony = getInput<double>("positiony").value();
    auto positionz = getInput<double>("positionz").value();
    auto roll = getInput<double>("roll").value();
    auto pitch = getInput<double>("pitch").value();
    auto yaw = getInput<double>("yaw").value();

    geometry_msgs::msg::Pose pose;
    pose.position.x = positionx;
    pose.position.y = positiony;
    pose.position.z = positionz;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    auto state = moveit_cpp_->getCurrentState();
    auto jmg = state->getJointModelGroup(planning_group_);
    bool found_ik = false;
    auto transform = state->getFrameTransform(planning_frame, &found_ik);
    if (!found_ik)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Could not find transform from planning frame to model frame.");
      return BT::NodeStatus::FAILURE;
    }

    Eigen::Isometry3d target_pose;
    tf2::fromMsg(pose, target_pose);

    // From planning frame to model frame
    target_pose = transform * target_pose;
    pose = tf2::toMsg(target_pose);
    RCLCPP_INFO(node_->get_logger(), "%s", geometry_msgs::msg::to_yaml(pose).c_str());

    if (!state->setFromIK(jmg, target_pose, moving_link))
    {
      RCLCPP_ERROR(node_->get_logger(), "Could not find IK solution.");
      return BT::NodeStatus::FAILURE;
    }

    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;
    collision_request.contacts = false;
    collision_request.cost = false;
    collision_request.distance = false;

    {
      auto planning_scene_monitor =
          planning_scene_monitor::LockedPlanningSceneRO(moveit_cpp_->getPlanningSceneMonitor());
      planning_scene_monitor->checkCollision(collision_request, collision_result, *state);
    }

    if (collision_result.collision)
    {
      RCLCPP_ERROR(node_->get_logger(), "Collision detected, could not set pose.");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("robot_state", state);
    return BT::NodeStatus::SUCCESS;
  }
};

#endif  // PLAN_MOTION_NODE_HPP
