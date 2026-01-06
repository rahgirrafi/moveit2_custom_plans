# MoveIt2 Custom Plans Workspace - Setup & Usage Guide

This guide covers setting up, building, and extending this MoveIt2 workspace with custom ROS 2 packages.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Importing Packages from .repos File](#importing-packages-from-repos-file)
3. [Building the Workspace](#building-the-workspace)
4. [Adding Custom ROS 2 Packages](#adding-custom-ros-2-packages)
   - [ament_cmake Package (C++)](#ament_cmake-package-c)
   - [ament_python Package (Python)](#ament_python-package-python)
5. [Tips & Best Practices](#tips--best-practices)
6. [Troubleshooting](#troubleshooting)

---

## Prerequisites

Before starting, ensure you have the following installed:

```bash
# ROS 2 Humble (or your target distribution)
# Check: https://docs.ros.org/en/humble/Installation.html

# Install vcstool for importing repositories
sudo apt update
sudo apt install python3-vcstool

# Install colcon build tools
sudo apt install python3-colcon-common-extensions

# Install rosdep for dependency resolution
sudo apt install python3-rosdep
sudo rosdep init  # Only run once
rosdep update
```

---

## Importing Packages from .repos File

The workspace uses a `.repos` file to manage external dependencies. This file defines Git repositories to clone into your workspace.

### Step 1: Navigate to Your Workspace

```bash
cd ~/ws_moveit/src
```

### Step 2: Import Repositories Using VCS

```bash
# Import all repositories defined in moveit2.repos
vcs import --recursive < src/moveit2_custom_plans/moveit2.repos #it's not strictly necessary for the .repos file to be inside a ros package. it can be anywhere.
```

This will clone the following packages:
- **moveit2** - Core MoveIt2 library
- **moveit_task_constructor** - Task planning framework
- **moveit_visual_tools** - Visualization utilities
- **moveit_msgs** - MoveIt message definitions
- **moveit_resources** - Sample robot configurations
- **ros2_kortex** - Kinova robot drivers
- **ros2_robotiq_gripper** - Robotiq gripper drivers
- **serial** - Serial communication library
- **rviz_visual_tools** - RViz visualization tools
- **vision_3D** - 3D vision integration

### Step 3: Update Existing Repositories (Optional)

```bash
# Pull latest changes for all repositories
vcs pull src
```

### Step 4: Check Repository Status

```bash
# View status of all repositories
vcs status src
```

---

## Building the Workspace

### Step 1: Install Dependencies with rosdep

```bash
cd ~/ws_moveit

# Install all dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Step 2: Build the Workspace

```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Build all packages
colcon build --symlink-install
```



**Build Options:**
You don't need to build the whole worksapce everytime you make some modifications 
```bash
# Build specific package only
colcon build --packages-select custom_plan

# Build package with its dependencies
colcon build --packages-up-to custom_plan

# Parallel build with limited jobs (useful for memory-constrained systems)
colcon build --parallel-workers 4

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Build in Release mode for better performance
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Step 3: Source the Workspace

```bash
source install/setup.bash
```

> **Tip:** Add this to your `~/.bashrc` for automatic sourcing:
> ```bash
> echo "source ~/ws_moveit/install/setup.bash" >> ~/.bashrc
> ```

---

## Adding Custom ROS 2 Packages

### ament_cmake Package (C++)

Create a C++ package with predefined nodes:

```bash
cd ~/ws_moveit/src

# Create package with dependencies
ros2 pkg create --build-type ament_cmake \
    --node-name my_node \
    --dependencies rclcpp moveit_ros_planning_interface \
    my_cpp_package
```

**Package Structure:**
```
my_cpp_package/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── my_cpp_package/
├── src/
│   └── my_node.cpp
└── launch/
```

**Example CMakeLists.txt with Multiple Nodes:**

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Add executables
add_executable(my_node src/my_node.cpp)
add_executable(another_node src/another_node.cpp)

# Include directories
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(another_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Set C++ standard
target_compile_features(my_node PUBLIC cxx_std_17)
target_compile_features(another_node PUBLIC cxx_std_17)

# Link dependencies
ament_target_dependencies(my_node rclcpp moveit_ros_planning_interface)
ament_target_dependencies(another_node rclcpp moveit_ros_planning_interface)

# Install executables
install(TARGETS my_node another_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files (if any)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
```

**Example Node Template (src/my_node.cpp):**

```cpp
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    RCLCPP_INFO(this->get_logger(), "MyNode initialized");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MyNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
```

---

### ament_python Package (Python)

Create a Python package with predefined nodes:

```bash
cd ~/ws_moveit/src

# Create Python package
ros2 pkg create --build-type ament_python \
    --node-name my_python_node \
    --dependencies rclpy \
    my_python_package
```

**Package Structure:**
```
my_python_package/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── my_python_package
├── my_python_package/
│   ├── __init__.py
│   └── my_python_node.py
├── launch/
└── config/
```

**Example setup.py with Multiple Nodes:**

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_python_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My custom Python package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_python_node = my_python_package.my_python_node:main',
            'another_node = my_python_package.another_node:main',
            'data_processor = my_python_package.data_processor:main',
        ],
    },
)
```

**Example Node Template (my_python_package/my_python_node.py):**
refer to the official documentaion for details: 
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyPythonNode(Node):
    def __init__(self):
        super().__init__('my_python_node')
        self.get_logger().info('MyPythonNode initialized')
        
        # Create a timer
        self.timer = self.create_timer(1.0, self.timer_callback)
    
    def timer_callback(self):
        self.get_logger().info('Timer callback triggered')


def main(args=None):
    rclpy.init(args=args)
    node = MyPythonNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Tips & Best Practices

### 1. Workspace Organization

```
ws_moveit/
├── src/
│   ├── external/          # Third-party packages (from .repos)
│   ├── my_packages/       # Your custom packages
│   └── moveit2_custom_plans/
├── build/                 # Build artifacts (auto-generated)
├── install/               # Installed packages (auto-generated)
└── log/                   # Build logs (auto-generated)
```

### 2. Creating Launch Files

**Python Launch File (launch/my_launch.launch.py):**

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('my_package')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
    ])
```

### 4. Cleaning the Workspace

```bash
# Remove build artifacts for a fresh build
rm -rf build/ install/ log/
```

### 5. Running Nodes

```bash
# Run a single node
ros2 run custom_plan plan_node

# Run with launch file
ros2 launch custom_plan plan_node.launch.py

# Run with parameters
ros2 run my_package my_node --ros-args -p param_name:=value
```

### 6. Useful Commands

```bash
# List all packages in workspace
colcon list

# Check package dependencies
ros2 pkg xml my_package --tag depend

# View node information
ros2 node info /my_node

# Echo topics
ros2 topic echo /my_topic

# Call services
ros2 service call /my_service std_srvs/srv/Trigger
```

---

## Troubleshooting

### Common Issues

**1. Missing Dependencies**
```bash
# Re-run rosdep
rosdep install --from-paths src --ignore-src -r -y
```

**2. Build Failures**
```bash
# Check for specific package errors
colcon build --packages-select problematic_package --event-handlers console_direct+
```

**3. Source Not Found**
```bash
# Ensure workspace is sourced after building
source ~/ws_moveit/install/setup.bash
```

**4. VCS Import Issues**
```bash
# Check for authentication issues
vcs import src < src/moveit2_custom_plans/moveit2.repos --debug
```

**5. Memory Issues During Build**
```bash
# Limit parallel jobs
colcon build --parallel-workers 2 --executor sequential
```

---

## Quick Reference Card

| Task | Command |
|------|---------|
| Import repos | `vcs import src < src/moveit2_custom_plans/moveit2.repos` |
| Update repos | `vcs pull src` |
| Install deps | `rosdep install --from-paths src --ignore-src -r -y` |
| Build all | `colcon build --symlink-install` |
| Build one | `colcon build --packages-select <pkg>` |
| Source | `source install/setup.bash` |
| Run node | `ros2 run <pkg> <node>` |
| Launch | `ros2 launch <pkg> <launch_file>` |
| Clean | `rm -rf build/ install/ log/` |
---

## Contributing
When adding new packages or modifying existing ones:
1. Follow ROS 2 naming conventions
2. Add appropriate dependencies to `package.xml`
3. Update `CMakeLists.txt` or `setup.py` accordingly
4. Create meaningful launch files
5. Document your nodes and their parameters

---

*Last updated: January 2026*

