#include "ur5e_bin_picking/bt_nodes.hpp"

namespace ur5e_bin_picking {

BT::PortsList SelectBestCandidate::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseArray>("detected_poses"),
        BT::OutputPort<geometry_msgs::msg::Pose>("target_pose"),
    };
}

BT::NodeStatus SelectBestCandidate::tick() {
    geometry_msgs::msg::PoseArray poses;
    if (!getInput("detected_poses", poses) || poses.poses.empty()) {
        return BT::NodeStatus::FAILURE;
    }
    // Selection policy: take the first (closest/topmost) object
    auto target = poses.poses[0];
    setOutput("target_pose", target);
    return BT::NodeStatus::SUCCESS;
}

}  // namespace
