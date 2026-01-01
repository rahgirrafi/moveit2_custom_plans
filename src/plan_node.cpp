/**
 * @file plan_node.cpp
 * @brief MoveIt Task Constructor node for button pushing with Kinova Gen3 robot
 *
 * This node implements a button-pushing task using the MoveIt Task Constructor (MTC).
 * It subscribes to a target pose topic, generates approach/push/retract motions,
 * and executes them on the robot.
 *
 * @author Custom Plan Team
 * @date 2026
 */

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <atomic>

#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_task_node");
namespace mtc = moveit::task_constructor;

// Robot configuration
constexpr const char* ARM_GROUP = "manipulator";
constexpr const char* GRIPPER_GROUP = "gripper";
constexpr const char* HAND_FRAME = "end_effector_link";
constexpr const char* TARGET_POSE_TOPIC = "/button/target_pose";

// Task parameters
constexpr double APPROACH_DISTANCE_MIN = 0.01;
constexpr double APPROACH_DISTANCE_MAX = 0.05;
constexpr double PUSH_DISTANCE_MIN = 0.02;
constexpr double PUSH_DISTANCE_MAX = 0.05;
constexpr double RETRACT_DISTANCE_MIN = 0.03;
constexpr double RETRACT_DISTANCE_MAX = 0.05;
constexpr double IK_FRAME_OFFSET_Z = 0.25;
constexpr int MAX_IK_SOLUTIONS = 32;
constexpr double MIN_IK_SOLUTION_DISTANCE = 0.1;
constexpr double CARTESIAN_STEP_SIZE = 0.002;
constexpr double CONNECT_TIMEOUT = 5.0;
constexpr int PLANNING_ATTEMPTS = 5;
}  // namespace

/**
 * @class MTCTaskNode
 * @brief ROS2 node for executing MTC button-pushing tasks
 */
class MTCTaskNode {
public:
  explicit MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void setupPlanningScene();
  void doTask();
  void centroidCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  geometry_msgs::msg::PoseStamped last_centroid_;
  std::atomic<bool> new_pose_received_{false};

private:
  mtc::Task createTask();
  void initializeSubscriber();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

// ============================================================================
// Implementation
// ============================================================================

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
    : node_(std::make_shared<rclcpp::Node>("mtc_custom_node", options)) {
  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  initializeSubscriber();
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface() {
  return node_->get_node_base_interface();
}

void MTCTaskNode::initializeSubscriber() {
  pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      TARGET_POSE_TOPIC, 10,
      std::bind(&MTCTaskNode::centroidCallback, this, std::placeholders::_1));
}

void MTCTaskNode::centroidCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(LOGGER, "Received target pose: [%.3f, %.3f, %.3f] in frame '%s'",
              msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
              msg->header.frame_id.c_str());
  last_centroid_ = *msg;
  new_pose_received_ = true;
}

void MTCTaskNode::setupPlanningScene() {
  // Create cylinder collision object for the button
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.02, 0.02};  // height, radius

  // Apply 180° rotation around X-axis to flip the object's Z-axis
  geometry_msgs::msg::Pose pose = last_centroid_.pose;
  tf2::Quaternion q_original, q_flip, q_result;
  tf2::fromMsg(pose.orientation, q_original);
  q_flip.setRPY(M_PI, 0, 0);
  q_result = q_original * q_flip;
  q_result.normalize();
  pose.orientation = tf2::toMsg(q_result);
  object.pose = pose;

  // Add object to planning scene
  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  RCLCPP_INFO(LOGGER, "Added collision object at [%.3f, %.3f, %.3f]",
              pose.position.x, pose.position.y, pose.position.z);

  // Publish TF frame for visualization in RViz
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = "world";
  transform.child_frame_id = "object_frame";
  transform.transform.translation.x = pose.position.x;
  transform.transform.translation.y = pose.position.y;
  transform.transform.translation.z = pose.position.z;
  transform.transform.rotation = pose.orientation;
  tf_broadcaster_->sendTransform(transform);
}

void MTCTaskNode::doTask() {
  task_ = createTask();

  // Initialize task
  try {
    task_.init();
  } catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR(LOGGER, "Task initialization failed: %s", e.what());
    return;
  }

  // Plan task
  RCLCPP_INFO(LOGGER, "Planning task...");
  if (!task_.plan(PLANNING_ATTEMPTS)) {
    RCLCPP_ERROR(LOGGER, "Task planning failed");
    task_.printState();
    return;
  }

  task_.printState();
  task_.introspection().publishSolution(*task_.solutions().front());

  // Execute task
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Task execution failed with code: %d", result.val);
    return;
  }

  RCLCPP_INFO(LOGGER, "Task executed successfully");
}

mtc::Task MTCTaskNode::createTask() {
  mtc::Task task;
  task.stages()->setName("push button task");
  task.loadRobotModel(node_);
  task.setProperty("group", ARM_GROUP);
  task.setProperty("eef", GRIPPER_GROUP);
  task.setProperty("ik_frame", HAND_FRAME);

  // Configure planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(CARTESIAN_STEP_SIZE);

  // Stage: Current State
  mtc::Stage* current_state_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::CurrentState>("current");
    current_state_ptr = stage.get();
    task.add(std::move(stage));
  }

  // Stage: Close Gripper
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close gripper", interpolation_planner);
    stage->setGroup(GRIPPER_GROUP);
    stage->setGoal("Close");
    task.add(std::move(stage));
  }

  // Stage: Connect (move to approach position)
  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to button",
        mtc::stages::Connect::GroupPlannerVector{{ARM_GROUP, sampling_planner}});
    stage->setTimeout(CONNECT_TIMEOUT);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage));
  }

  // Serial Container: Push Sequence
  {
    auto container = std::make_unique<mtc::SerialContainer>("push button");
    task.properties().exposeTo(container->properties(), {"eef", "group", "ik_frame"});
    container->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    // Generate approach pose with IK
    {
      auto generator = std::make_unique<mtc::stages::GenerateGraspPose>("generate push pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->properties().set("marker_ns", "push_pose");
      generator->setPreGraspPose("Close");
      generator->setObject("object");
      generator->setAngleDelta(M_PI / 12);
      generator->setMonitoredStage(current_state_ptr);

      Eigen::Isometry3d ik_frame_transform = Eigen::Isometry3d::Identity();
      ik_frame_transform.translation().z() = IK_FRAME_OFFSET_Z;

      auto ik_wrapper = std::make_unique<mtc::stages::ComputeIK>("push pose IK", std::move(generator));
      ik_wrapper->setMaxIKSolutions(MAX_IK_SOLUTIONS);
      ik_wrapper->setMinSolutionDistance(MIN_IK_SOLUTION_DISTANCE);
      ik_wrapper->setIgnoreCollisions(false);
      ik_wrapper->setIKFrame(ik_frame_transform, HAND_FRAME);
      ik_wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      ik_wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      container->insert(std::move(ik_wrapper));
    }

    // Allow collision: gripper <-> object
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,object)");
      stage->allowCollisions(
          "object",
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)->getLinkModelNamesWithCollisionGeometry(),
          true);
      container->insert(std::move(stage));
    }

    // Allow collision: gripper <-> octomap
    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (gripper,octomap)");
      stage->allowCollisions(
          "octomap",
          task.getRobotModel()->getJointModelGroup(GRIPPER_GROUP)->getLinkModelNamesWithCollisionGeometry(),
          true);
      container->insert(std::move(stage));
    }

    // Approach: move toward button
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(APPROACH_DISTANCE_MIN, APPROACH_DISTANCE_MAX);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "approach");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = HAND_FRAME;
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      container->insert(std::move(stage));
    }

    // Push: press the button
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("push", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(PUSH_DISTANCE_MIN, PUSH_DISTANCE_MAX);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "push");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = HAND_FRAME;
      direction.vector.z = 1.0;
      stage->setDirection(direction);
      container->insert(std::move(stage));
    }

    // Retract: move away from button
    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retract", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(RETRACT_DISTANCE_MIN, RETRACT_DISTANCE_MAX);
      stage->setIKFrame(HAND_FRAME);
      stage->properties().set("marker_ns", "retract");

      geometry_msgs::msg::Vector3Stamped direction;
      direction.header.frame_id = HAND_FRAME;
      direction.vector.z = -1.0;
      stage->setDirection(direction);
      container->insert(std::move(stage));
    }

    task.add(std::move(container));
  }

  return task;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  RCLCPP_INFO(LOGGER, "Starting MTC Task Node...");

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto mtc_node = std::make_shared<MTCTaskNode>(options);

  // Spin executor in background thread
  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_node]() {
    executor.add_node(mtc_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_node->getNodeBaseInterface());
  });

  // Main loop: wait for target poses and execute tasks
  while (rclcpp::ok()) {
    RCLCPP_INFO(LOGGER, "Waiting for %s...", TARGET_POSE_TOPIC);

    while (rclcpp::ok() && !mtc_node->new_pose_received_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!rclcpp::ok()) break;

    mtc_node->new_pose_received_ = false;
    RCLCPP_INFO(LOGGER, "Target pose received, executing task...");

    try {
      mtc_node->setupPlanningScene();
      mtc_node->doTask();
      RCLCPP_INFO(LOGGER, "Task completed successfully.");
    } catch (const mtc::InitStageException& e) {
      RCLCPP_ERROR(LOGGER, "Task init failed: %s", e.what());
    } catch (const std::exception& e) {
      RCLCPP_ERROR(LOGGER, "Task exception: %s", e.what());
    }
  }

  RCLCPP_INFO(LOGGER, "Shutting down...");
  executor.cancel();
  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}