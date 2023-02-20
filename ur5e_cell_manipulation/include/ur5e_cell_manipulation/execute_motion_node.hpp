#ifndef EXECUTE_MOTION_NODE_HPP
#define EXECUTE_MOTION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <future>

class ExecuteMotionNode : public BT::StatefulActionNode
{
protected:
  rclcpp::Node::SharedPtr node_;
  moveit_cpp::MoveItCppPtr moveit_cpp_;
  std::string planning_group_;
  std::future<moveit_controller_manager::ExecutionStatus> fut_;

public:
  ExecuteMotionNode(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    RCLCPP_INFO(node_->get_logger(), "Initialising Execute Node.");
    moveit_cpp_ = config.blackboard->get<moveit_cpp::MoveItCppPtr>("moveit_cpp");
    planning_group_ = config.blackboard->get<std::string>("planning_group");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<robot_trajectory::RobotTrajectoryPtr>("trajectory") };
  }

  BT::NodeStatus onStart() override
  {
    RCLCPP_INFO(node_->get_logger(), "Initialising motion execution");
    if (getInput<robot_trajectory::RobotTrajectoryPtr>("trajectory").has_value() == false)
    {
      RCLCPP_ERROR(node_->get_logger(), "Exception caught: %s",
                   getInput<robot_trajectory::RobotTrajectoryPtr>("trajectory").error().c_str());
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      return BT::NodeStatus::FAILURE;
    }
    robot_trajectory::RobotTrajectoryPtr trajectory =
        getInput<robot_trajectory::RobotTrajectoryPtr>("trajectory").value();

    fut_ = std::async(std::launch::async, [this, &trajectory]() {
      auto status = moveit_cpp_->execute(planning_group_, trajectory);
      RCLCPP_INFO(node_->get_logger(), "Executing motion status: %d", status);
      return status;
    });
    RCLCPP_INFO(node_->get_logger(), "Executing motion.");
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::future_status status = fut_.wait_for(std::chrono::milliseconds(1));
    switch (status)
    {
      case std::future_status::ready:
        if (fut_.get() == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
        {
          RCLCPP_INFO(node_->get_logger(), "Executing motion success.");
          return BT::NodeStatus::SUCCESS;
        }
        else
        {
          RCLCPP_INFO(node_->get_logger(), "Executing motion failure.");
          return BT::NodeStatus::FAILURE;
        }
      default:
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
  }

  void onHalted() override
  {
    RCLCPP_INFO(node_->get_logger(), "Halting motion");
    moveit_cpp_->getTrajectoryExecutionManager()->stopExecution();
  }
};

#endif  // EXECUTE_MOTION_NODE_HPP
