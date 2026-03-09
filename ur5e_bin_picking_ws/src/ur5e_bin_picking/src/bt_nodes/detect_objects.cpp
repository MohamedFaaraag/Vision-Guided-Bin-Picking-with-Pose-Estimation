#include "ur5e_bin_picking/bt_nodes.hpp"

namespace ur5e_bin_picking {

DetectObjects::DetectObjects(const std::string& name, const BT::NodeConfig& config)
    : BT::StatefulActionNode(name, config)
{
    node_ = rclcpp::Node::make_shared("bt_detect_objects");
    sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "/object_poses_base", 10,
        [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
            latest_poses_ = *msg;
            received_ = true;
        });
}

BT::PortsList DetectObjects::providedPorts() {
    return { BT::OutputPort<geometry_msgs::msg::PoseArray>("detected_poses") };
}

BT::NodeStatus DetectObjects::onStart() {
    received_ = false;
    start_time_ = node_->get_clock()->now();
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectObjects::onRunning() {
    rclcpp::spin_some(node_);
    if (received_ && !latest_poses_.poses.empty()) {
        setOutput("detected_poses", latest_poses_);
        RCLCPP_INFO(node_->get_logger(), "Detected %zu objects", latest_poses_.poses.size());
        return BT::NodeStatus::SUCCESS;
    }
    auto elapsed = (node_->get_clock()->now() - start_time_).seconds();
    if (elapsed > 5.0) {
        RCLCPP_WARN(node_->get_logger(), "Detection timeout");
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

void DetectObjects::onHalted() { received_ = false; }

}  // namespace
