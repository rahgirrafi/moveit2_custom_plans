#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
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

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_task_node");
namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void doTask();
  void setupPlanningScene();
  geometry_msgs::msg::PoseStamped last_centroid_;
  void centroidCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
  
  // One-shot centroid subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr centroid_sub_;
  void subscribeToCentroidOnce();
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_custom_node", options) }
{
   subscribeToCentroidOnce();
}

void MTCTaskNode::subscribeToCentroidOnce() {
  centroid_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/button/target_pose", 10,
    std::bind(&MTCTaskNode::centroidCallback, this, std::placeholders::_1)
  );
}

void MTCTaskNode::centroidCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  RCLCPP_INFO(LOGGER, "Target point for MoveTo stage: [%.3f, %.3f, %.3f, %.3f] Frame: %s", 
                msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, msg->pose.orientation.w, msg->header.frame_id.c_str());
  last_centroid_ = *msg;
  RCLCPP_INFO(LOGGER, "Target point for MoveTo stage: [%.3f, %.3f, %.3f, %.3f] Frame: %s", 
                last_centroid_.pose.position.x, last_centroid_.pose.position.y, last_centroid_.pose.position.z, last_centroid_.pose.orientation.w, last_centroid_.header.frame_id.c_str());
  centroid_sub_.reset();
}


void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.02, 0.02 };

  geometry_msgs::msg::Pose pose;
  pose = last_centroid_.pose;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
  
  RCLCPP_INFO(LOGGER, "Added collision object to planning scene");
  
  // NOTE: Octomap collision allowances should be added to the SRDF file directly
  // Publishing an ACM via /planning_scene REPLACES the entire ACM, which would 
  // remove all the disable_collisions entries from the SRDF
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    RCLCPP_INFO(LOGGER, "Initializing MTC task...");
    task_.init();
    RCLCPP_INFO(LOGGER, "MTC task initialized successfully.");
  }
    catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR(LOGGER, "=== DETAILED INIT STAGE EXCEPTION ===");
    RCLCPP_ERROR(LOGGER, "What: %s", e.what());
    RCLCPP_ERROR_STREAM(LOGGER, "Full details: " << e);
    RCLCPP_ERROR(LOGGER, "=====================================");
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning MTC task...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    task_.printState();  // Print task state summary
    return;
  }
  
  // Always print task state summary after planning
  RCLCPP_INFO(LOGGER, "=== Task Planning Summary ===");
  task_.printState();
  RCLCPP_INFO(LOGGER, "=============================");
  
  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << result.val);
    return;
  }
  RCLCPP_INFO(LOGGER, "Task executed successfully.");
}


mtc::Task MTCTaskNode::createTask()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("push button task");
  task.loadRobotModel(node_);
  
  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "end_effector_link";
  
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  RCLCPP_INFO(LOGGER, "Adding stage: %s", stage_state_current->name().c_str());
  task.add(std::move(stage_state_current));
  
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.002);  // Reduced from 0.01 to get more waypoints for short distances



  //add stages here 

  /*
  Stages to add:
  Current State (Generator Stage) (current state)
  Open Gripper (Propagator) (move to)
  MoveToPush (Connector) 
    Approach Button (Propagator Stage) (MoveRelative)
    Allow COllision between object and gripper (Propagator) (ModifyPlanningScene)
    Push Pose(Generator Stage) (GenerateGraspPose) (PoseStamped or Eigen Tranformation Matrix) (ComputeIK from object to gripper using the transform) (This stage determines how far the gripper will stop from the button. Basically this is the push)
    [close gripper (Propagator Stage) (MoveTo)] {if not needed then ignore}
    Pull Back (Propagator Stage) (MoveRelative)
  */

  // Stage: Move arm to Home position
  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("Become Vertical", interpolation_planner);
    stage->setGroup(arm_group_name);
    stage->setGoal("Vertical");
    task.add(std::move(stage));
  }

  // Stage: Close gripper for pushing (keep it closed throughout)
  {
    auto stage =
        std::make_unique<mtc::stages::MoveTo>("close hand for push", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("Close");
    task.add(std::move(stage));
  }

  // Stage: Connect - plans arm motion from current position to pre-push position
  auto stage_move_to_push = std::make_unique<mtc::stages::Connect>(
      "move to button",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  stage_move_to_push->setTimeout(5.0);
  stage_move_to_push->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_push));

  {
    auto push_container = std::make_unique<mtc::SerialContainer>("push button");
    task.properties().exposeTo(push_container->properties(), { "eef", "group", "ik_frame" });
   
    push_container->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
   
    // Stage 1: Generate push pose + Compute IK (GENERATOR - central stage)
    // This generates the "approach start" position - farther from the button
    // All Cartesian motions will be AFTER this stage (forward propagating)
    {
      auto stage = std::make_unique<mtc::stages::GenerateGraspPose>("generate push pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "push_pose");
      stage->setPreGraspPose("Close");  // Gripper stays closed for pushing
      stage->setObject("object");
      stage->setAngleDelta(M_PI / 12);
      stage->setMonitoredStage(current_state_ptr);
      
      // Transform: end-effector positioned at approach start (farther from button)
      // This is where Cartesian motion begins - must be collision-free
      Eigen::Isometry3d push_frame_transform = Eigen::Isometry3d::Identity();
      push_frame_transform.translation().z() = 0.25;  // 25cm before button (approach start)

      // Compute IK - ensure collision-free poses for Connect stage
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("push pose IK", std::move(stage));
      wrapper->setMaxIKSolutions(32);
      wrapper->setMinSolutionDistance(0.1);
      wrapper->setIgnoreCollisions(false);
      wrapper->setIKFrame(push_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      push_container->insert(std::move(wrapper));
    }

    // Stage 2: Allow collision between gripper and button (before approach)
    {
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup("gripper")
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      push_container->insert(std::move(stage));
    }

    // Stage 3: APPROACH - Cartesian motion toward the button (straight line)
    // Move from 25cm to ~5cm before the button
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.15, 0.20);  // Approach distance: 15-20cm
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "approach");

      // Approach along positive Z (toward button) - straight line Cartesian motion
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      push_container->insert(std::move(stage));
    }

    // Stage 4: PUSH - Continue Cartesian motion to press the button
    // Move from ~5cm to contact with button
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("push", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.02, 0.05);  // Push distance: 2-5cm (button is small)
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "push");

      // Push forward along positive Z (into the button)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      push_container->insert(std::move(stage));
    }

    // Stage 5: RETRACT - Cartesian motion backward after pressing
    {
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("Return to Verti", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.20, 0.30);  // Retract distance: 20-30cm (back to safe position)
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retract");

      // Retract along negative Z (back away from button)
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = -1.0;
      stage->setDirection(vec);
      push_container->insert(std::move(stage));
    }

    // Note: We don't re-enable collision checking since the task ends here

    task.add(std::move(push_container));
  }
  
  return task;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(LOGGER, "Starting MTC Task Node...");
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->getNodeBaseInterface());
    executor.spin();
    executor.remove_node(mtc_task_node->getNodeBaseInterface());
  });
  
  RCLCPP_INFO(LOGGER, "Waiting for centroid message...");
  while (rclcpp::ok() && mtc_task_node->last_centroid_.header.frame_id.empty()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  mtc_task_node->setupPlanningScene();
  try {
    RCLCPP_INFO(LOGGER, "Calling doTask()...");
    
    mtc_task_node->doTask();
  }
  catch (const mtc::InitStageException& e) {
    RCLCPP_ERROR(LOGGER, "=== DETAILED INIT STAGE EXCEPTION ===");
    RCLCPP_ERROR(LOGGER, "What: %s", e.what());
    RCLCPP_ERROR(LOGGER, "=====================================");
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Caught exception: %s", e.what());
    
  }

  RCLCPP_INFO(LOGGER, "Waiting for spin thread to join...");
  spin_thread->join();
  rclcpp::shutdown();
  RCLCPP_INFO(LOGGER, "MTC Task Node shutdown complete.");
  return 0;
}