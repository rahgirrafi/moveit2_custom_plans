# MTC IK Solution Debugging Guide

This document describes the Inverse Kinematics (IK) problems encountered during the development of the MoveIt Task Constructor button push task, and the solutions applied.

---

## Table of Contents

1. [IK Solutions Being Filtered Out](#1-ik-solutions-being-filtered-out)
2. [Zero IK Solutions from Generator Stage](#2-zero-ik-solutions-from-generator-stage)
3. [Connect Stage Failing to Find Path](#3-connect-stage-failing-to-find-path)
4. [Start State Out of Bounds](#4-start-state-out-of-bounds)
5. [Start State in Collision](#5-start-state-in-collision)
6. [Cartesian Path Min Fraction Not Met](#6-cartesian-path-min-fraction-not-met)
7. [Summary of Key Parameters](#7-summary-of-key-parameters)

---

## 1. IK Solutions Being Filtered Out

### Problem

The `ComputeIK` stage was generating IK solutions but they were all being filtered out, resulting in 0 solutions propagating to the next stage.

**Error Pattern:**

```
push pose IK (0/25):
  generate push pose: 25 solutions found
  (but 0 solutions after IK)
```

### Root Cause

The `setMinSolutionDistance()` parameter was set too high (1.0 radians). This filters out IK solutions that are "too similar" to each other in joint space. With a high threshold, nearly all solutions were being discarded.

### Solution

Reduce the minimum solution distance:

```cpp
// BEFORE (too restrictive):
wrapper->setMinSolutionDistance(1.0);

// AFTER (allows more solutions):
wrapper->setMinSolutionDistance(0.1);
```

### Explanation

- `setMinSolutionDistance(distance)` filters IK solutions to ensure diversity
- A value of 1.0 rad means solutions must differ by at least 57° in joint space
- A value of 0.1 rad (5.7°) allows more similar solutions through
- For manipulation tasks, 0.1-0.2 is typically appropriate

---

## 2. Zero IK Solutions from Generator Stage

### Problem

The generator stage (`GenerateGraspPose` or `GeneratePlacePose`) was producing poses, but `ComputeIK` was returning 0 valid solutions.

**Error Pattern:**

```
push pose IK (0/X): no IK solution found
```

### Possible Causes and Solutions

#### 2.1 Wrong IK Frame Transform

The `setIKFrame()` transform positions the end-effector relative to the target object. If this is wrong, the resulting poses may be unreachable.

```cpp
// Ensure the transform makes sense for your task
Eigen::Isometry3d push_frame_transform = Eigen::Isometry3d::Identity();
push_frame_transform.translation().z() = 0.25;  // 25cm approach distance

wrapper->setIKFrame(push_frame_transform, "end_effector_link");
```

#### 2.2 Insufficient IK Solutions Requested

```cpp
// Increase the number of IK solutions to try
wrapper->setMaxIKSolutions(32);  // Default might be 8
```

#### 2.3 Collisions Blocking All Solutions

If `setIgnoreCollisions(false)`, all IK solutions might be in collision:

```cpp
// For debugging, try ignoring collisions temporarily
wrapper->setIgnoreCollisions(true);  // DEBUGGING ONLY

// If this works, the issue is collision configuration
```

#### 2.4 Object Not in Reachable Workspace

The target object may be outside the robot's reachable workspace. Check:

- Object position relative to robot base
- Robot arm reach limits
- Consider different approach angles via `setAngleDelta()`

```cpp
// Generate more approach angles
stage->setAngleDelta(M_PI / 12);  // 15° increments = 24 poses around object
```

---

## 3. Connect Stage Failing to Find Path

### Problem

The `Connect` stage couldn't find a valid motion plan between the current state and the IK solution.

**Error Pattern:**

```
move to button (0/X): No motion plan found
```

### Possible Causes and Solutions

#### 3.1 Gripper State Mismatch

The Connect stage plans from the **current robot state** to the **IK solution state**. If the gripper state differs, planning may fail.

```cpp
// WRONG: Gripper is open at start, but IK expects closed
// Connect stage can't plan because gripper would need to move

// SOLUTION: Add a gripper stage BEFORE Connect
{
  auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage->setGroup("gripper");
  stage->setGoal("Close");
  task.add(std::move(stage));
}

// Then add Connect stage
auto connect = std::make_unique<mtc::stages::Connect>("move to button", ...);
```

#### 3.2 Case Sensitivity in Named States

SRDF state names are **case-sensitive**:

```cpp
// WRONG:
stage->setGoal("close");  // lowercase

// CORRECT:
stage->setGoal("Close");  // matches SRDF exactly
```

#### 3.3 Stage Ordering Issue

`ModifyPlanningScene` stages must come AFTER generator stages, not before:

```cpp
// WRONG ORDER:
// 1. ModifyPlanningScene (allow collision)  <-- No state to modify yet!
// 2. ComputeIK (generator)

// CORRECT ORDER:
// 1. ComputeIK (generator)  <-- Generates states
// 2. ModifyPlanningScene    <-- Modifies the generated states
```

#### 3.4 Planner Timeout Too Short

```cpp
stage_move_to_button->setTimeout(5.0);  // Increase if needed
```

---

## 4. Start State Out of Bounds

### Problem

The current robot state has joint values at or beyond the URDF limits.

**Error Pattern:**

```
Become Vertical (0/1): Start state is out of bounds!
```

### Diagnosis

Check current joint states:

```bash
ros2 topic echo /joint_states --once
```

Compare with URDF limits:

```bash
ros2 param get /move_group robot_description | grep -oP 'joint_[0-9].*?limit.*?/>'
```

### Solutions

#### 4.1 Extend URDF Joint Limits

Add a small buffer to the URDF joint limits:

**File:** `kortex_description/arms/gen3/7dof/urdf/gen3_macro.xacro`

```xml
<!-- BEFORE -->
<limit lower="-2.24" upper="2.24" effort="39" velocity="1.3963" />

<!-- AFTER (added 0.03 rad buffer) -->
<limit lower="-2.27" upper="2.27" effort="39" velocity="1.3963" />
```

**Joints extended:**
| Joint | Original | Extended | Buffer |
|-------|----------|----------|--------|
| joint_2 | ±2.24 | ±2.27 | ±0.03 |
| joint_4 | ±2.57 | ±2.60 | ±0.03 |
| joint_6 | ±2.09 | ±2.12 | ±0.03 |

#### 4.2 Extend Gripper Joint Limits

**File:** `robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro`

```xml
<!-- BEFORE -->
<limit lower="0.0" upper="0.8" ... />

<!-- AFTER -->
<limit lower="-0.01" upper="0.81" ... />
```

#### 4.3 Adjust SRDF Named States

Ensure named states are slightly inside limits:

**File:** `gen3.srdf`

```xml
<!-- BEFORE (at exact limits) -->
<group_state name="Open" group="gripper">
  <joint name="robotiq_85_left_knuckle_joint" value="0.0" />
</group_state>

<!-- AFTER (inside limits) -->
<group_state name="Open" group="gripper">
  <joint name="robotiq_85_left_knuckle_joint" value="0.001" />
</group_state>
```

---

## 5. Start State in Collision

### Problem

The robot's current configuration has self-collisions or collisions with the environment.

**Error Pattern:**

```
Become Vertical (0/1): Start state is in collision!
```

### Diagnosis

In RViz, the Motion Planning panel shows colliding link pairs in red.

### Solutions

#### 5.1 Gripper Self-Collisions (Mesh Overlap)

The Robotiq 2F-85 gripper meshes overlap at certain positions. Add `disable_collisions` entries to the SRDF:

**File:** `gen3.srdf`

```xml
<!-- Gripper finger link pairs -->
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

#### 5.2 ACM Being Overwritten

**Critical Issue:** Publishing a `PlanningScene` message with an ACM **replaces** the entire ACM, removing all SRDF disable_collisions entries.

```cpp
// WRONG - This overwrites the entire ACM:
moveit_msgs::msg::PlanningScene planning_scene_msg;
planning_scene_msg.is_diff = true;
planning_scene_msg.allowed_collision_matrix.entry_names = {"link1", "link2"};
// ...
publisher->publish(planning_scene_msg);

// CORRECT - Don't publish ACM, use SRDF disable_collisions instead
// Or use PlanningSceneMonitor to get and modify the existing ACM
```

#### 5.3 Octomap Collisions

If using depth cameras, the octomap may cause false collisions. Add to SRDF:

```xml
<disable_collisions link1="base_link" link2="<octomap>" reason="Never"/>
<disable_collisions link1="shoulder_link" link2="<octomap>" reason="Never"/>
<!-- ... for all robot links that touch the mounting surface -->
```

---

## 6. Cartesian Path Min Fraction Not Met

### Problem

The `MoveRelative` stage with Cartesian planner couldn't complete the requested motion distance.

**Error Pattern:**

```
push (0/78): CartesianPath: min_fraction not met. Achieved: 0.392157
min_distance not reached (0.0392 < 0.05)
```

### Root Cause

The planner hit an obstacle (collision object, joint limit, or singularity) before completing the minimum requested distance.

### Solutions

#### 6.1 Reduce Minimum Distance

```cpp
// BEFORE (required 5-10cm):
stage->setMinMaxDistance(0.05, 0.10);

// AFTER (allows 2-5cm):
stage->setMinMaxDistance(0.02, 0.05);
```

#### 6.2 Adjust Approach Distance

If approaching too close before the push stage:

```cpp
// Reduce approach distance to leave room for push
stage->setMinMaxDistance(0.10, 0.15);  // Less approach = more room for push
```

#### 6.3 Check IK Frame Position

The IK frame transform determines where Cartesian motion starts:

```cpp
// If push starts too close to object, reduce this value
push_frame_transform.translation().z() = 0.20;  // Start farther back
```

---

## 7. Summary of Key Parameters

### ComputeIK Stage

| Parameter                  | Default | Recommended | Effect                   |
| -------------------------- | ------- | ----------- | ------------------------ |
| `setMaxIKSolutions()`      | 8       | 32          | More solutions to try    |
| `setMinSolutionDistance()` | 0.1     | 0.1-0.2     | Filter similar solutions |
| `setIgnoreCollisions()`    | false   | false       | Enable for debugging     |

### GenerateGraspPose / GeneratePlacePose

| Parameter           | Default | Recommended    | Effect                                |
| ------------------- | ------- | -------------- | ------------------------------------- |
| `setAngleDelta()`   | π/6     | π/12           | More approach angles (smaller = more) |
| `setPreGraspPose()` | -       | "Open"/"Close" | Gripper state at target               |

### MoveRelative (Cartesian)

| Parameter             | Default | Recommended  | Effect                  |
| --------------------- | ------- | ------------ | ----------------------- |
| `setMinMaxDistance()` | -       | (0.02, 0.05) | Minimum motion required |

### Connect Stage

| Parameter      | Default | Recommended | Effect                |
| -------------- | ------- | ----------- | --------------------- |
| `setTimeout()` | 1.0     | 5.0         | Planning time allowed |

---

## Debugging Workflow

1. **Check the failing stage name** in the MTC output
2. **Look at the numbers**: `(success/total)` shows how many solutions passed
3. **Trace backwards**: If stage N has 0/X, check what stage N-1 produced
4. **Check joint states**: `ros2 topic echo /joint_states --once`
5. **Check RViz**: Motion Planning panel shows collision pairs
6. **Increase verbosity**: Add `RCLCPP_INFO` statements in your code
7. **Test incrementally**: Comment out later stages to isolate issues

---

## Common MTC Output Interpretation

```
push button task
  1  - ←   1 →   -  0 / current              # 1 solution (current state)
  -  0 →   1 →   -  0 / Become Vertical      # 0 failed, 1 succeeded forward
  -  0 →   1 →   -  1 / close hand           # 1 succeeded, 1 pending backward
  -  1 →   0 ←   0  - / move to button       # 1 pending forward, 0 succeeded
  0  - ←   0 →   -  0 / push button          # 0 solutions = FAILURE HERE
```

**Reading the arrows:**

- `→` = forward propagation
- `←` = backward propagation
- Numbers show: `(pending) - (failed) → (succeeded)`

**Key insight:** If a later stage has 0 successes, check if earlier stages are providing valid input states.
