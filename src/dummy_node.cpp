#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


#include <moveit/robot_state/conversions.h>

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
  

private:
  mtc::Task createTask();
  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;
};

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>("mtc_custom_node", options) }
{
}

void MTCTaskNode::doTask()
{
  task_ = createTask();

  try
  {
    RCLCPP_INFO(LOGGER, "Initializing MTC task...");
    task_.init();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task initialization failed: " << e);
    return;
  }

  RCLCPP_INFO(LOGGER, "Planning MTC task...");
  if (!task_.plan(5))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }
  RCLCPP_INFO(LOGGER, "Publishing solution...");
  task_.introspection().publishSolution(*task_.solutions().front());

  RCLCPP_INFO(LOGGER, "Executing solution...");
  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed with code: " << result.val);
    return;
  }
  RCLCPP_INFO(LOGGER, "Task executed successfully.");
  return;
}
mtc::Task MTCTaskNode::createTask()
{
  moveit::task_constructor::Task task;
  task.stages()->setName("push button task");
  task.loadRobotModel(node_);
  
  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "gripper";
  
  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  
  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  RCLCPP_INFO(LOGGER, "Adding stage: %s", stage_state_current->name().c_str());
  task.add(std::move(stage_state_current));
  
  // Planners
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(.01);
  
  // Open gripper
  auto stage_look = std::make_unique<mtc::stages::MoveTo>("look arm", interpolation_planner);
  stage_look->setGroup(arm_group_name);
  stage_look->setGoal("Look");
  RCLCPP_INFO(LOGGER, "Adding stage: %s", stage_look->name().c_str());
  task.add(std::move(stage_look));

  // Open gripper
  auto stage_retract = std::make_unique<mtc::stages::MoveTo>("retract arm", interpolation_planner);
  stage_retract->setGroup(arm_group_name);
  stage_retract->setGoal("Retract");
  RCLCPP_INFO(LOGGER, "Adding stage: %s", stage_retract->name().c_str());
  task.add(std::move(stage_retract));
  

  {
  // Rotate base 360 degrees using target pose
  auto stage_rotate_base = std::make_unique<mtc::stages::MoveTo>(
    "rotate base 360 degrees", 
    interpolation_planner);
  stage_rotate_base->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  
  geometry_msgs::msg::PointStamped target_point;
  
  
  stage_rotate_base->setGoal(target_point);
  
  RCLCPP_INFO(LOGGER, "Adding stage: %s", stage_rotate_base->name().c_str());
  task.add(std::move(stage_rotate_base));
}
  

// { 
//   auto stage_joint_relative = std::make_unique<mtc::stages::MoveRelative>( "Rotate 360 degrees", interpolation_planner); 
//   stage_joint_relative->properties().configureInitFrom( mtc::Stage::PARENT, { "group" }); 
//   std::map<std::string, double> joint_deltas; 
//   joint_deltas["joint_3"] = M_PI /2; 
//   stage_joint_relative->setDirection(joint_deltas); 
//   RCLCPP_INFO(LOGGER, "Adding stage: %s", 
//   stage_joint_relative->name().c_str()); 
//   task.add(std::move(stage_joint_relative)); 
// }
//   RCLCPP_INFO(LOGGER, "Task creation complete with %zu stages", task.stages()->numChildren());
//   return task;
// }

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

  RCLCPP_INFO(LOGGER, "Calling doTask()...");
  mtc_task_node->doTask();

  RCLCPP_INFO(LOGGER, "Waiting for spin thread to join...");
  spin_thread->join();
  rclcpp::shutdown();
  RCLCPP_INFO(LOGGER, "MTC Task Node shutdown complete.");
  return 0;
}