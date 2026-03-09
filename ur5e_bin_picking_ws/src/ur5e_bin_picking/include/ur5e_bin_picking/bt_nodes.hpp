#pragma once

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/task_constructor/task.h>

namespace ur5e_bin_picking
{

// ── DetectObjects: Waits for /object_poses_base message ──
class DetectObjects : public BT::StatefulActionNode
{
public:
    DetectObjects(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_;
    geometry_msgs::msg::PoseArray latest_poses_;
    bool received_ = false;
    rclcpp::Time start_time_;
};

// ── SelectBestCandidate: Picks the first/best detected object ──
class SelectBestCandidate : public BT::SyncActionNode
{
public:
    SelectBestCandidate(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ── PlanGrasp: Creates MTC task for the selected object ──
class PlanGrasp : public BT::StatefulActionNode
{
public:
    PlanGrasp(const std::string& name, const BT::NodeConfig& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    rclcpp::Node::SharedPtr node_;
    bool planning_complete_ = false;
    bool planning_success_ = false;
};

// ── ExecutePick: Executes the planned MTC pick task ──
class ExecutePick : public BT::StatefulActionNode
{
public:
    ExecutePick(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    bool execution_started_ = false;
    bool execution_complete_ = false;
    bool execution_success_ = false;
};

// ── VerifyGrasp: Checks if object is held (gripper feedback) ──
class VerifyGrasp : public BT::SyncActionNode
{
public:
    VerifyGrasp(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// ── ExecutePlace: Executes the place portion of the task ──
class ExecutePlace : public BT::StatefulActionNode
{
public:
    ExecutePlace(const std::string& name, const BT::NodeConfig& config)
        : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

// ── CheckBinEmpty: Returns FAILURE if more objects exist ──
class CheckBinEmpty : public BT::SyncActionNode
{
public:
    CheckBinEmpty(const std::string& name, const BT::NodeConfig& config)
        : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace ur5e_bin_picking
