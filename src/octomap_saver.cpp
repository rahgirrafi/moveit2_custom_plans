// Octomap Save/Load Utility for MoveIt
// This node provides services to save and load the planning scene (including octomap)

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#include <fstream>
#include <filesystem>

class OctomapSaver : public rclcpp::Node
{
public:
  OctomapSaver() : Node("octomap_saver")
  {
    // Declare parameters
    this->declare_parameter("save_directory", "/tmp/moveit_octomaps");
    this->declare_parameter("default_filename", "octomap");
    
    save_directory_ = this->get_parameter("save_directory").as_string();
    default_filename_ = this->get_parameter("default_filename").as_string();
    
    // Create save directory if it doesn't exist
    std::filesystem::create_directories(save_directory_);
    
    // Publisher for loading scenes
    planning_scene_pub_ = this->create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10);
    
    // Subscriber for octomap
    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "/planning_scene_world/octomap",
        10,
        std::bind(&OctomapSaver::octomapCallback, this, std::placeholders::_1));
    
    // Services
    save_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/save_octomap",
        std::bind(&OctomapSaver::saveCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    load_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/load_octomap",
        std::bind(&OctomapSaver::loadCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    clear_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/clear_octomap",
        std::bind(&OctomapSaver::clearCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Octomap Saver initialized");
    RCLCPP_INFO(this->get_logger(), "  Save directory: %s", save_directory_.c_str());
    RCLCPP_INFO(this->get_logger(), "Services:");
    RCLCPP_INFO(this->get_logger(), "  ~/save_octomap - Save current octomap");
    RCLCPP_INFO(this->get_logger(), "  ~/load_octomap - Load saved octomap");
    RCLCPP_INFO(this->get_logger(), "  ~/clear_octomap - Clear the current octomap");
    RCLCPP_INFO(this->get_logger(), "Listening for octomap on: /planning_scene_world/octomap");
  }

private:
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    latest_octomap_ = *msg;
    has_octomap_ = true;
    RCLCPP_DEBUG(this->get_logger(), "Received octomap with %zu bytes", msg->data.size());
  }

  void saveCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(octomap_mutex_);
    
    if (!has_octomap_ || latest_octomap_.data.empty()) {
      response->success = false;
      response->message = "No octomap available to save. Make sure 3D sensors are publishing.";
      RCLCPP_WARN(this->get_logger(), "No octomap available to save");
      return;
    }
    
    try {
      // Convert message to octomap
      octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(latest_octomap_);
      if (!abstract_tree) {
        response->success = false;
        response->message = "Failed to convert octomap message";
        return;
      }
      
      octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(abstract_tree);
      if (!tree) {
        delete abstract_tree;
        response->success = false;
        response->message = "Octomap is not an OcTree type";
        return;
      }
      
      // Generate filename with timestamp
      auto now = std::chrono::system_clock::now();
      auto time_t = std::chrono::system_clock::to_time_t(now);
      std::stringstream ss;
      ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
      
      std::string filename = save_directory_ + "/" + default_filename_ + "_" + ss.str() + ".bt";
      std::string latest_filename = save_directory_ + "/" + default_filename_ + "_latest.bt";
      
      // Save to binary file
      if (tree->writeBinary(filename)) {
        // Copy to latest
        std::filesystem::copy_file(filename, latest_filename, 
                                   std::filesystem::copy_options::overwrite_existing);
        
        response->success = true;
        response->message = "Octomap saved to: " + filename;
        RCLCPP_INFO(this->get_logger(), "Saved octomap to: %s", filename.c_str());
      } else {
        response->success = false;
        response->message = "Failed to write octomap file";
      }
      
      delete tree;
      
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Error saving: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Error saving octomap: %s", e.what());
    }
  }

  void loadCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      std::string filename = save_directory_ + "/" + default_filename_ + "_latest.bt";
      
      if (!std::filesystem::exists(filename)) {
        response->success = false;
        response->message = "No saved octomap found at: " + filename;
        RCLCPP_WARN(this->get_logger(), "No saved octomap found at: %s", filename.c_str());
        return;
      }
      
      // Load octomap from file
      octomap::OcTree* tree = new octomap::OcTree(filename);
      if (!tree) {
        response->success = false;
        response->message = "Failed to load octomap from file";
        return;
      }
      
      // Convert to message
      octomap_msgs::msg::Octomap octomap_msg;
      octomap_msgs::binaryMapToMsg(*tree, octomap_msg);
      octomap_msg.header.frame_id = "world";
      octomap_msg.header.stamp = this->now();
      
      // Create planning scene message with the octomap
      moveit_msgs::msg::PlanningScene scene_msg;
      scene_msg.is_diff = true;
      scene_msg.world.octomap.header = octomap_msg.header;
      scene_msg.world.octomap.octomap = octomap_msg;
      scene_msg.world.octomap.origin.orientation.w = 1.0;
      
      planning_scene_pub_->publish(scene_msg);
      
      delete tree;
      
      response->success = true;
      response->message = "Octomap loaded from: " + filename;
      RCLCPP_INFO(this->get_logger(), "Loaded octomap from: %s", filename.c_str());
      
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Error loading: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Error loading octomap: %s", e.what());
    }
  }

  void clearCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      // Create a planning scene diff that clears the octomap
      moveit_msgs::msg::PlanningScene scene_msg;
      scene_msg.is_diff = true;
      scene_msg.world.octomap.header.frame_id = "world";
      scene_msg.world.octomap.header.stamp = this->now();
      scene_msg.world.octomap.octomap.id = "OcTree";
      scene_msg.world.octomap.octomap.binary = true;
      scene_msg.world.octomap.octomap.resolution = 0.05;
      scene_msg.world.octomap.octomap.data.clear();  // Empty data clears octomap
      scene_msg.world.octomap.origin.orientation.w = 1.0;
      
      planning_scene_pub_->publish(scene_msg);
      
      response->success = true;
      response->message = "Octomap cleared";
      RCLCPP_INFO(this->get_logger(), "Octomap cleared");
      
    } catch (const std::exception& e) {
      response->success = false;
      response->message = std::string("Error clearing: ") + e.what();
      RCLCPP_ERROR(this->get_logger(), "Error clearing octomap: %s", e.what());
    }
  }

  std::string save_directory_;
  std::string default_filename_;
  
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_service_;
  
  std::mutex octomap_mutex_;
  octomap_msgs::msg::Octomap latest_octomap_;
  bool has_octomap_ = false;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<OctomapSaver>();
  
  RCLCPP_INFO(node->get_logger(), "Starting Octomap Saver node...");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
