#include "ur5e_bin_picking/bt_nodes.hpp"

namespace ur5e_bin_picking {

PlanGrasp::PlanGrasp(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("bt_plan_grasp");
}

BT::PortsList PlanGrasp::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::Pose>("target_pose"),
        BT::OutputPort<bool>("plan_success"),
    };
}

BT::NodeStatus PlanGrasp::onStart() {
    planning_complete_ = false;
    planning_success_ = false;
    // In a full implementation, this would create the MTC task
    // and call task.plan(10) asynchronously
    RCLCPP_INFO(node_->get_logger(), "Planning grasp for target object...");
    planning_complete_ = true;
    planning_success_ = true;
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanGrasp::onRunning() {
    if (planning_complete_) {
        setOutput("plan_success", planning_success_);
        return planning_success_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void PlanGrasp::onHalted() {}

}  // namespace
