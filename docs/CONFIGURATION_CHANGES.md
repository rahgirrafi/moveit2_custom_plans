# Configuration Changes for MTC Button Push Task

This document summarizes all configuration changes made to fix issues with the MoveIt Task Constructor button push task for the Kinova Gen3 7-DOF robot with Robotiq 2F-85 gripper.

---

## 1. URDF Joint Limit Extensions

### File: `src/ros2_kortex/kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro`

Extended revolute joint limits to prevent "Start state is out of bounds!" errors when the robot is at or near its joint limits.

| Joint   | Original Limits (rad) | New Limits (rad) | Buffer Added |
| ------- | --------------------- | ---------------- | ------------ |
| joint_2 | [-2.24, 2.24]         | [-2.27, 2.27]    | ±0.03 rad    |
| joint_4 | [-2.57, 2.57]         | [-2.60, 2.60]    | ±0.03 rad    |
| joint_6 | [-2.09, 2.09]         | [-2.12, 2.12]    | ±0.03 rad    |

**Note:** joint_1, joint_3, joint_5, and joint_7 are continuous joints (no limits).

---

## 2. URDF Gripper Joint Limit Extensions

### File: `src/ros2_robotiq_gripper/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro`

Extended gripper joint limits to prevent boundary issues:

| Joint                          | Original Limits | New Limits    |
| ------------------------------ | --------------- | ------------- |
| robotiq_85_left_knuckle_joint  | [0.0, 0.8]      | [-0.01, 0.81] |
| robotiq_85_right_knuckle_joint | [-0.8, 0.0]     | [-0.81, 0.01] |

---

## 3. SRDF Gripper State Values

### File: `src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/gen3.srdf`

Changed gripper state values to be slightly inside the joint limits:

| State | Original Value | New Value |
| ----- | -------------- | --------- |
| Open  | 0.0            | 0.001     |
| Close | 0.8            | 0.79      |

---

## 4. SRDF Disable Collisions (Gripper Self-Collisions)

### File: `src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/gen3.srdf`

Added `<disable_collisions>` entries for gripper link pairs that have overlapping meshes:

```xml
<!-- Gripper self-collision pairs -->
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_left_inner_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_left_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_right_inner_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_base_link" link2="robotiq_85_right_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_left_finger_tip_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_left_finger_link" link2="robotiq_85_left_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_left_finger_tip_link" link2="robotiq_85_left_inner_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_right_finger_link" link2="robotiq_85_right_finger_tip_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_right_finger_link" link2="robotiq_85_right_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="robotiq_85_right_finger_tip_link" link2="robotiq_85_right_inner_knuckle_link" reason="Adjacent"/>
<disable_collisions link1="bracelet_link" link2="robotiq_85_base_link" reason="Adjacent"/>
```

---

## 5. Plan Node ACM Fix

### File: `src/custom_plan/src/plan_node.cpp`

**Problem:** The `setupPlanningScene()` function was publishing an Allowed Collision Matrix (ACM) via `/planning_scene` that **replaced** the entire ACM instead of merging with it. This deleted all the `disable_collisions` entries from the SRDF.

**Fix:** Removed the ACM publishing code. The SRDF's disable_collisions entries are now preserved.

```cpp
// BEFORE (problematic):
moveit_msgs::msg::PlanningScene planning_scene_msg;
planning_scene_msg.is_diff = true;
planning_scene_msg.allowed_collision_matrix.entry_names = all_names;
// ... (building ACM with only 9 entries)
planning_scene_diff_publisher->publish(planning_scene_msg);

// AFTER (fixed):
// Removed ACM publishing - use SRDF disable_collisions instead
```

---

## 6. Push Stage Distance Adjustment

### File: `src/custom_plan/src/plan_node.cpp`

Reduced the minimum push distance to allow shorter push motions:

```cpp
// BEFORE:
stage->setMinMaxDistance(0.05, 0.10);  // Required 5-10cm push

// AFTER:
stage->setMinMaxDistance(0.02, 0.05);  // Allows 2-5cm push
```

**Reason:** The Cartesian planner could only achieve ~4cm before hitting the button collision object, failing the 5cm minimum requirement.

---

## Summary of Issues Fixed

| Issue | Error Message                      | Root Cause                                | Fix                                     |
| ----- | ---------------------------------- | ----------------------------------------- | --------------------------------------- |
| 1     | "Start state is out of bounds!"    | Gripper at exact joint limits             | Changed SRDF states to be inside limits |
| 2     | "Start state is out of bounds!"    | Arm joints at exact limits                | Extended URDF joint limits              |
| 3     | "Start state is in collision!"     | Gripper mesh self-intersections           | Added disable_collisions to SRDF        |
| 4     | Collisions after running plan_node | ACM overwritten by planning scene publish | Removed ACM publishing code             |
| 5     | "min_fraction not met"             | Push distance too long                    | Reduced minimum push distance           |

---

## Rebuild Commands

After making these changes, rebuild the affected packages:

```bash
cd ~/ws_moveit_forked
source /opt/ros/humble/setup.bash

# Rebuild description packages
colcon build --packages-select kortex_description robotiq_description

# Rebuild MoveIt config
colcon build --packages-select kinova_gen3_7dof_robotiq_2f_85_moveit_config

# Rebuild custom_plan
colcon build --packages-select custom_plan

# Source and restart move_group
source install/setup.bash
```

**Important:** Always restart `move_group` after URDF/SRDF changes to reload the robot description.
