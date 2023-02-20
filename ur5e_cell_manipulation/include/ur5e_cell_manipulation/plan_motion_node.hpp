#ifndef PLAN_MOTION_NODE_HPP
#define PLAN_MOTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

class PlanMotionNode : public BT::SyncActionNode
{
protected:
  rclcpp::Node::SharedPtr node_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  moveit_cpp::PlanningComponentPtr planning_component_;
  std::string planning_group_;
  bool first_;

public:
  PlanMotionNode(const std::string& name, const BT::NodeConfiguration& config)
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
             BT::InputPort<moveit::core::RobotStatePtr>("start_state"),
             BT::InputPort<moveit::core::RobotStatePtr>("goal_state"),
             BT::InputPort<std::string>("planning_frame"),
             BT::InputPort<std::string>("moving_link"),
             BT::OutputPort<robot_trajectory::RobotTrajectoryPtr>("trajectory"),
             BT::OutputPort<moveit::core::RobotStatePtr>("trajectory_goal_state") };
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(node_->get_logger(), "Planning motion");

    if (getInput<moveit::core::RobotStatePtr>("start_state").has_value())
    {
      planning_component_->setStartState(
          *(getInput<moveit::core::RobotStatePtr>("start_state").value()));
    }
    else
    {
      planning_component_->setStartStateToCurrentState();
    }

    std::string planning_frame = getInput<std::string>("planning_frame").value();
    std::string link = getInput<std::string>("moving_link").value();

    if (getInput<moveit::core::RobotStatePtr>("goal_state").has_value())
    {
      planning_component_->setGoal(*(getInput<moveit::core::RobotStatePtr>("goal_state").value()));
    }
    else
    {
      double position_x = getInput<double>("positionx").value();
      double position_y = getInput<double>("positiony").value();
      double position_z = getInput<double>("positionz").value();
      double roll = getInput<double>("roll").value();
      double pitch = getInput<double>("pitch").value();
      double yaw = getInput<double>("yaw").value();
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = planning_frame;
      pose.pose.position.x = position_x;
      pose.pose.position.y = position_y;
      pose.pose.position.z = position_z;
      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);
      pose.pose.orientation.x = q.x();
      pose.pose.orientation.y = q.y();
      pose.pose.orientation.z = q.z();
      pose.pose.orientation.w = q.w();
      planning_component_->setGoal(pose, link);
    }

    auto plan = planning_component_->plan();

    if (!plan)
    {
      RCLCPP_ERROR(node_->get_logger(), "Planning failed");
      return BT::NodeStatus::FAILURE;
    }
    setOutput("trajectory", plan.trajectory);
    setOutput("trajectory_goal_state", plan.trajectory->getLastWayPointPtr());
    return BT::NodeStatus::SUCCESS;
  }
};

#endif  // PLAN_MOTION_NODE_HPP
