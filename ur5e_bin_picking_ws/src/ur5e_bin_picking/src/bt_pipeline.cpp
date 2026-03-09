/**
 * Phase 5 — BehaviorTree Pipeline Orchestrator
 * Loads behavior_tree.xml, registers all custom BT action nodes,
 * and ticks the tree in a loop until the bin is empty or an error occurs.
 */
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "ur5e_bin_picking/bt_nodes.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("bt_pipeline");

    // Get tree file path from parameter or default
    std::string tree_file;
    node->declare_parameter("tree_file", "");
    node->get_parameter("tree_file", tree_file);

    if (tree_file.empty()) {
        tree_file = ament_index_cpp::get_package_share_directory("ur5e_bin_picking")
            + "/config/behavior_tree.xml";
    }

    RCLCPP_INFO(node->get_logger(), "Loading behavior tree from: %s", tree_file.c_str());

    // ── Register all custom BT nodes ──
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<ur5e_bin_picking::DetectObjects>("DetectObjects");
    factory.registerNodeType<ur5e_bin_picking::SelectBestCandidate>("SelectBestCandidate");
    factory.registerNodeType<ur5e_bin_picking::PlanGrasp>("PlanGrasp");
    factory.registerNodeType<ur5e_bin_picking::ExecutePick>("ExecutePick");
    factory.registerNodeType<ur5e_bin_picking::VerifyGrasp>("VerifyGrasp");
    factory.registerNodeType<ur5e_bin_picking::ExecutePlace>("ExecutePlace");
    factory.registerNodeType<ur5e_bin_picking::CheckBinEmpty>("CheckBinEmpty");

    // ── Load and run the tree ──
    auto tree = factory.createTreeFromFile(tree_file);

    RCLCPP_INFO(node->get_logger(), "Behavior tree loaded. Starting autonomous pipeline...");

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (status == BT::NodeStatus::RUNNING && rclcpp::ok()) {
        status = tree.tickOnce();
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node->get_logger(), "Pipeline finished with status: %s",
        BT::toStr(status).c_str());

    rclcpp::shutdown();
    return 0;
}
