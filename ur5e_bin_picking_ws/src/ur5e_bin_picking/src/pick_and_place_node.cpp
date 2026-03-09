/**
 * Phase 3/4 — MTC Pick-and-Place Node
 *
 * Subscribes to /object_poses_base (PoseArray in base_link frame).
 * For each detected object, creates an MTC pick-and-place task:
 *   Stage 1: CurrentState
 *   Stage 2: Open Gripper
 *   Stage 3: Move to Pick (Connect)
 *   Stage 4: Pick (SerialContainer: Approach, GenerateGraspPose, ComputeIK,
 *            AllowCollision, Close Gripper, Attach, Lift)
 *   Stage 5: Move to Place (Connect)
 *   Stage 6: Place (SerialContainer: Lower, GeneratePlacePose, ComputeIK,
 *            Open Gripper, Detach, Retreat)
 *   Stage 7: Return Home
 * Plans multiple solutions and executes the best (lowest cost).
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/solvers.h>

namespace mtc = moveit::task_constructor;

class PickAndPlaceNode : public rclcpp::Node
{
public:
    PickAndPlaceNode()
        : Node("pick_and_place_node",
               rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        // Subscribe to transformed object poses from Phase 2
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/object_poses_base", 10,
            std::bind(&PickAndPlaceNode::pose_callback, this, std::placeholders::_1));

        // Place location (configurable)
        place_pose_.header.frame_id = "base_link";
        place_pose_.pose.position.x = 0.4;
        place_pose_.pose.position.y = -0.3;
        place_pose_.pose.position.z = 0.05;
        place_pose_.pose.orientation.w = 1.0;

        RCLCPP_INFO(this->get_logger(), "Pick-and-Place node ready");
    }

    void setup_planning_scene()
    {
        moveit::planning_interface::PlanningSceneInterface psi;

        // Add table collision object
        moveit_msgs::msg::CollisionObject table;
        table.id = "table";
        table.header.frame_id = "base_link";
        table.primitives.resize(1);
        table.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        table.primitives[0].dimensions = {1.0, 1.0, 0.02};
        table.primitive_poses.resize(1);
        table.primitive_poses[0].position.x = 0.4;
        table.primitive_poses[0].position.y = 0.0;
        table.primitive_poses[0].position.z = -0.01;
        table.primitive_poses[0].orientation.w = 1.0;
        table.operation = moveit_msgs::msg::CollisionObject::ADD;

        // Add bin collision object
        moveit_msgs::msg::CollisionObject bin;
        bin.id = "bin";
        bin.header.frame_id = "base_link";
        bin.primitives.resize(1);
        bin.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        bin.primitives[0].dimensions = {0.30, 0.20, 0.10};
        bin.primitive_poses.resize(1);
        bin.primitive_poses[0].position.x = 0.4;
        bin.primitive_poses[0].position.y = 0.0;
        bin.primitive_poses[0].position.z = 0.05;
        bin.primitive_poses[0].orientation.w = 1.0;
        bin.operation = moveit_msgs::msg::CollisionObject::ADD;

        psi.applyCollisionObjects({table, bin});
        RCLCPP_INFO(this->get_logger(), "Planning scene initialized with table and bin");
    }

    mtc::Task createPickPlaceTask(
        const geometry_msgs::msg::PoseStamped& object_pose,
        const std::string& object_id)
    {
        mtc::Task task;
        task.stages()->setName("pick_place_task");
        task.loadRobotModel(this->shared_from_this());

        // ── Planners ──
        auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
        cartesian_planner->setMaxVelocityScalingFactor(1.0);
        cartesian_planner->setMaxAccelerationScalingFactor(1.0);
        cartesian_planner->setStepSize(0.01);

        auto pipeline_planner = std::make_shared<mtc::solvers::PipelinePlanner>();
        pipeline_planner->setPlannerId("RRTConnect");

        auto joint_interpolation =
            std::make_shared<mtc::solvers::JointInterpolationPlanner>();

        // ── Stage 1: Current State ──
        auto current_state = std::make_unique<mtc::stages::CurrentState>("current");
        task.add(std::move(current_state));

        // ── Stage 2: Open Gripper ──
        auto open_gripper = std::make_unique<mtc::stages::MoveTo>(
            "open_gripper", joint_interpolation);
        open_gripper->setGroup("gripper");
        open_gripper->setGoal("open");
        task.add(std::move(open_gripper));

        // ── Stage 3: Move to Pick ──
        auto move_to_pick = std::make_unique<mtc::stages::Connect>(
            "move_to_pick",
            mtc::stages::Connect::GroupPlannerVector{
                {"ur_manipulator", pipeline_planner}});
        task.add(std::move(move_to_pick));

        // ── Stage 4: Pick (SerialContainer) ──
        {
            auto pick = std::make_unique<mtc::SerialContainer>("pick_object");

            // 4a: Approach (straight down)
            auto approach = std::make_unique<mtc::stages::MoveRelative>(
                "approach", cartesian_planner);
            approach->setGroup("ur_manipulator");
            geometry_msgs::msg::Vector3Stamped direction;
            direction.header.frame_id = "base_link";
            direction.vector.z = -1.0;
            approach->setDirection(direction);
            approach->setMinMaxDistance(0.05, 0.15);
            pick->insert(std::move(approach));

            // 4b: Generate Grasp Pose
            auto grasp_generator = std::make_unique<mtc::stages::GenerateGraspPose>(
                "generate_grasp_pose");
            grasp_generator->setMonitoredStage(task.stages()->findChild("current"));
            grasp_generator->setObject(object_id);
            grasp_generator->setAngleDelta(M_PI / 6);  // 30 degree increments

            // 4c: Compute IK wrapping the generator
            auto compute_ik = std::make_unique<mtc::stages::ComputeIK>(
                "compute_ik", std::move(grasp_generator));
            compute_ik->setGroup("ur_manipulator");
            compute_ik->setIKFrame("tool0");
            compute_ik->setMaxIKSolutions(8);
            pick->insert(std::move(compute_ik));

            // 4d: Allow collision between gripper and object
            auto allow_collision = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow_collision");
            allow_collision->allowCollisions(object_id,
                task.getRobotModel()->getJointModelGroup("gripper")
                    ->getLinkModelNamesWithCollisionGeometry(), true);
            pick->insert(std::move(allow_collision));

            // 4e: Close gripper
            auto close_gripper = std::make_unique<mtc::stages::MoveTo>(
                "close_gripper", joint_interpolation);
            close_gripper->setGroup("gripper");
            close_gripper->setGoal("closed");
            pick->insert(std::move(close_gripper));

            // 4f: Attach object to tool0
            auto attach = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "attach_object");
            attach->attachObject(object_id, "tool0");
            pick->insert(std::move(attach));

            // 4g: Lift (straight up)
            auto lift = std::make_unique<mtc::stages::MoveRelative>(
                "lift", cartesian_planner);
            lift->setGroup("ur_manipulator");
            geometry_msgs::msg::Vector3Stamped lift_dir;
            lift_dir.header.frame_id = "base_link";
            lift_dir.vector.z = 1.0;
            lift->setDirection(lift_dir);
            lift->setMinMaxDistance(0.10, 0.20);
            pick->insert(std::move(lift));

            task.add(std::move(pick));
        }

        // ── Stage 5: Move to Place ──
        auto move_to_place = std::make_unique<mtc::stages::Connect>(
            "move_to_place",
            mtc::stages::Connect::GroupPlannerVector{
                {"ur_manipulator", pipeline_planner}});
        task.add(std::move(move_to_place));

        // ── Stage 6: Place (SerialContainer) ──
        {
            auto place = std::make_unique<mtc::SerialContainer>("place_object");

            // 6a: Lower
            auto lower = std::make_unique<mtc::stages::MoveRelative>(
                "lower", cartesian_planner);
            lower->setGroup("ur_manipulator");
            geometry_msgs::msg::Vector3Stamped lower_dir;
            lower_dir.header.frame_id = "base_link";
            lower_dir.vector.z = -1.0;
            lower->setDirection(lower_dir);
            lower->setMinMaxDistance(0.05, 0.15);
            place->insert(std::move(lower));

            // 6b: Generate Place Pose
            auto place_generator = std::make_unique<mtc::stages::GeneratePlacePose>(
                "generate_place_pose");
            place_generator->setObject(object_id);
            place_generator->setPose(place_pose_);
            auto place_ik = std::make_unique<mtc::stages::ComputeIK>(
                "place_ik", std::move(place_generator));
            place_ik->setGroup("ur_manipulator");
            place_ik->setIKFrame("tool0");
            place->insert(std::move(place_ik));

            // 6c: Open gripper (release)
            auto open = std::make_unique<mtc::stages::MoveTo>(
                "release", joint_interpolation);
            open->setGroup("gripper");
            open->setGoal("open");
            place->insert(std::move(open));

            // 6d: Detach object
            auto detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach");
            detach->detachObject(object_id, "tool0");
            place->insert(std::move(detach));

            // 6e: Retreat (straight up)
            auto retreat = std::make_unique<mtc::stages::MoveRelative>(
                "retreat", cartesian_planner);
            retreat->setGroup("ur_manipulator");
            geometry_msgs::msg::Vector3Stamped retreat_dir;
            retreat_dir.header.frame_id = "base_link";
            retreat_dir.vector.z = 1.0;
            retreat->setDirection(retreat_dir);
            retreat->setMinMaxDistance(0.05, 0.15);
            place->insert(std::move(retreat));

            task.add(std::move(place));
        }

        // ── Stage 7: Return Home ──
        auto home = std::make_unique<mtc::stages::MoveTo>(
            "return_home", pipeline_planner);
        home->setGroup("ur_manipulator");
        home->setGoal("home");
        task.add(std::move(home));

        return task;
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (msg->poses.empty()) {
            RCLCPP_WARN(this->get_logger(), "No objects detected");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Received %zu objects, processing first", msg->poses.size());

        // Add detected object to planning scene
        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject obj;
        obj.id = "pick_object_0";
        obj.header.frame_id = "base_link";
        obj.primitives.resize(1);
        obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        obj.primitives[0].dimensions = {0.04, 0.02};  // height, radius
        obj.primitive_poses.resize(1);
        obj.primitive_poses[0] = msg->poses[0];
        obj.operation = moveit_msgs::msg::CollisionObject::ADD;
        psi.applyCollisionObject(obj);

        // Build pose for MTC
        geometry_msgs::msg::PoseStamped object_pose;
        object_pose.header.frame_id = "base_link";
        object_pose.pose = msg->poses[0];

        // Create and execute MTC task
        auto task = createPickPlaceTask(object_pose, "pick_object_0");
        task.init();

        if (task.plan(10)) {
            RCLCPP_INFO(this->get_logger(),
                "Found %zu solutions", task.solutions().size());
            task.execute(*task.solutions().front());
            RCLCPP_INFO(this->get_logger(), "Pick-and-place executed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "No MTC solutions found");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    geometry_msgs::msg::PoseStamped place_pose_;
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceNode>();
    node->setup_planning_scene();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
