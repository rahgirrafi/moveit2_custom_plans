# MoveIt Task Constructor (MTC) Debugging Notes

## Problem Summary

The MTC task was failing with multiple errors during development:

1. `grasp pose IK (0/25): 0.261799 no IK found` - IK solver couldn't find solutions
2. `Error initializing stage(s)` - Stage initialization failure
3. `Deviation in joint robotiq_85_left_inner_knuckle_joint: [0] != [0.662578]` - Connect stage joint mismatch

---

## Issue 1: IK Solutions Being Filtered Out

### Symptom

```
grasp pose IK (0/25): 0.261799 no IK found
```

The `GenerateGraspPose` stage generated 25 candidate poses, but `ComputeIK` found 0 valid solutions.

### Root Cause

```cpp
wrapper->setMinSolutionDistance(1.0);  // TOO HIGH!
```

**What `setMinSolutionDistance` does:**

- Specifies the minimum distance **in joint space (radians)** between accepted IK solutions
- A value of `1.0` means solutions must differ by at least 1 radian (~57°) per joint
- This is extremely restrictive for a 7-DOF arm and filters out nearly all valid solutions

### Fix

```cpp
wrapper->setMinSolutionDistance(0.1);  // Reduced - allows more solutions
```

---

## Issue 2: Incorrect Stage Order in SerialContainer

### Symptom

```
Error initializing stage(s). RCLCPP_ERROR_STREAM(e) for details.
```

### Root Cause

The stages in the `SerialContainer` were in the wrong order. In MTC, stages flow **from the Generator stage**:

- Stages **before** the generator propagate **backward** (from generator toward task start)
- Stages **after** the generator propagate **forward** (from generator toward task end)

**Wrong Order (before fix):**

```
1. approach button        ← MoveRelative (propagator)
2. allow collision        ← ModifyPlanningScene (propagator)  ❌ WRONG POSITION
3. grasp pose IK          ← ComputeIK wrapping GenerateGraspPose (GENERATOR)
4. close hand             ← MoveTo (propagator)
5. lift object            ← MoveRelative (propagator)
```

The `ModifyPlanningScene` stage (allow collision) was placed **before** the generator, but it should come **after** because:

- It enables collisions between the gripper and object
- This is needed **after** reaching the grasp pose, not before

**Correct Order (after fix):**

```
1. approach button        ← MoveRelative (backward propagating)
2. grasp pose IK          ← GENERATOR (central stage)
3. allow collision        ← ModifyPlanningScene (forward propagating)
4. close hand             ← MoveTo (forward propagating)
5. retreat                ← MoveRelative (forward propagating)
```

### Fix

Reordered stages so `ModifyPlanningScene` comes after the generator.

---

## Issue 3: Incorrect Gripper Goal State Name

### Symptom

Stage initialization failure when trying to close the gripper.

### Root Cause

```cpp
stage->setGoal("close");  // Wrong - case sensitive!
```

The SRDF defines the gripper states as:

```xml
<group_state name="Open" group="gripper">
<group_state name="Close" group="gripper">
```

### Fix

```cpp
stage->setGoal("Close");  // Match SRDF exactly (capital C)
```

---

## Issue 4: Retreat Direction

### Original Code

```cpp
// "lift object" stage - moving up in world Z
vec.header.frame_id = "world";
vec.vector.z = 1.0;
```

### Issue

For a button-pushing task, lifting straight up in world coordinates may not be the desired behavior. Retreating along the gripper's axis (backing away from the button) is more appropriate.

### Fix

```cpp
// "retreat" stage - backing away along gripper Z-axis
vec.header.frame_id = hand_frame;
vec.vector.z = -1.0;  // Negative Z = back away
```

---

## Issue 5: grasp_frame_transform Orientation

### Context

The `grasp_frame_transform` defines the spatial relationship between the **grasp pose** (at the object) and the **end-effector frame**.

### Original Code (caused issues)

```cpp
grasp_frame_transform.linear() = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitZ()).matrix();
grasp_frame_transform.translation().z() = 0.15;
```

### Problem

- The 90° rotation around Z combined with the object's orientation created unreachable poses
- When combined with the object's non-identity orientation `(x: 0.705, y: 0.013, z: 0.013, w: 0.708)`, the resulting end-effector poses hit joint limits

### Fix (simplified for debugging)

```cpp
grasp_frame_transform = Eigen::Isometry3d::Identity();
grasp_frame_transform.translation().z() = 0.15;  // Just offset, no rotation
```

Once basic functionality is confirmed, rotation can be added back based on the specific gripper orientation needed.

---

## Issue 6: Missing "Open Hand" Stage Before Connect

### Symptom

```
[Connect]: Deviation in joint robotiq_85_left_inner_knuckle_joint: [0] != [0.662578]
```

This message repeats many times, and the `Connect` stage fails to find a valid path.

### Root Cause

The `Connect` stage tries to merge two trajectory segments, but the **gripper joint states don't match** between them:

- **Start side** (from CurrentState): Gripper is closed (joint value = `0`)
- **End side** (from SerialContainer): `GenerateGraspPose` expects gripper in "Open" state (joint value = `0.662578`) due to `setPreGraspPose("Open")`

The `Connect` stage only plans for the **arm group** (`manipulator`), not the gripper. So if the gripper states differ on either side, the trajectories cannot be merged.

**Missing Stage:**
There was no stage to actually open the gripper before the Connect stage. The `setPreGraspPose("Open")` only sets the _expected_ state for IK computation - it doesn't move the gripper.

### Fix

Add an "open hand" stage **before** the Connect stage:

```cpp
// Stage: Open gripper - REQUIRED before Connect stage
// This ensures gripper state matches what GenerateGraspPose expects (setPreGraspPose)
{
  auto stage =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage->setGroup(hand_group_name);
  stage->setGoal("Open");
  task.add(std::move(stage));
}
```

### Correct Task Flow

```
current → Become Vertical → open hand → [Connect] → SerialContainer
                               ↑
                    Ensures gripper is "Open" before Connect
```

---

## MTC Stage Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                         TASK FLOW                                │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  CurrentState ──► Become Vertical ──► open hand ──► [Connect]   │
│       │                                                  │       │
│       │                                                  ▼       │
│       │            ┌─────────── SerialContainer ──────────┐     │
│       │            │                                       │     │
│       │            │   approach ◄── GENERATOR ──► allow    │     │
│       │            │   button       (IK)         collision │     │
│       │            │                    │                  │     │
│       │            │                    ▼                  │     │
│       │            │              close hand               │     │
│       │            │                    │                  │     │
│       │            │                    ▼                  │     │
│       │            │               retreat                 │     │
│       │            └───────────────────────────────────────┘     │
│       │                                                          │
│       └──────── monitored by GenerateGraspPose ─────────────────┘
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Key Lessons Learned

1. **Stage order matters**: In a `SerialContainer`, the Generator is the central stage. Propagators before it flow backward; propagators after it flow forward.

2. **Case sensitivity**: SRDF state names are case-sensitive (`"Close"` ≠ `"close"`).

3. **`setMinSolutionDistance`**: Keep this value low (0.05-0.1) to avoid filtering out valid IK solutions. Only increase if you specifically need diverse solutions.

4. **Debug with identity transforms first**: When troubleshooting IK issues, simplify the `grasp_frame_transform` to identity, then add rotations once basic functionality works.

5. **Check object reachability**: Ensure the object position is within the robot's workspace before debugging IK.

6. **Match gripper states across Connect stages**: The `Connect` stage only plans for specified groups. If the gripper state differs between the two sides being connected, add a stage to synchronize the gripper state before the Connect.

7. **`setPreGraspPose` vs actual gripper motion**: `setPreGraspPose("Open")` only tells the IK solver what gripper state to assume - it does NOT move the gripper. You need a separate `MoveTo` stage to actually open/close the gripper.

---

## Files Modified

- `/home/rafilappy/ws_moveit_forked/src/custom_plan/src/plan_node.cpp`
- `/home/rafilappy/ws_moveit_forked/src/ros2_kortex/kortex_moveit_config/kinova_gen3_7dof_robotiq_2f_85_moveit_config/config/gen3.srdf`

## Date

January 1, 2026

---

## Issue 7: Joint Values at Exact Limits Cause "Out of Bounds" Error

### Symptom

```
close hand for push (0/1): Start state is out of bounds!
```

The stage receives 1 solution but produces 0 valid outputs.

### Root Cause

The gripper joint values in the SRDF were set to **exactly** the joint limits:

| State | SRDF Value | URDF Limit   |
| ----- | ---------- | ------------ |
| Open  | `0.0`      | lower: `0.0` |
| Close | `0.8`      | upper: `0.8` |

Due to floating-point precision in trajectory interpolation (e.g., via `JointInterpolationPlanner`), the computed joint values can slightly exceed the exact limit (e.g., `0.800001`), causing MoveIt to reject the state as "out of bounds".

### Fix

Adjust SRDF values to be **slightly inside** the joint limits:

**Before:**

```xml
<group_state name="Open" group="gripper">
    <joint name="robotiq_85_left_knuckle_joint" value="0"/>
</group_state>
<group_state name="Close" group="gripper">
    <joint name="robotiq_85_left_knuckle_joint" value="0.8"/>
</group_state>
```

**After:**

```xml
<group_state name="Open" group="gripper">
    <joint name="robotiq_85_left_knuckle_joint" value="0.001"/>
</group_state>
<group_state name="Close" group="gripper">
    <joint name="robotiq_85_left_knuckle_joint" value="0.79"/>
</group_state>
```

### Key Takeaway

Always set SRDF joint values **slightly inside** the URDF limits (by ~0.01 rad or 1%) to avoid floating-point precision issues.

---

## How to Interpret MTC Output

### Output Format

```
   X  - ←  Y →   -  Z / stage_name
```

| Column  | Meaning                                                           |
| ------- | ----------------------------------------------------------------- |
| `X`     | Solutions received from **previous** stage (backward propagation) |
| `Y`     | Solutions **available** at this stage                             |
| `Z`     | Solutions sent to **next** stage (forward propagation)            |
| `←` `→` | Propagation direction arrows                                      |
| `-`     | Not applicable for this direction                                 |

### Example Analysis

```
1  - ←   1 →   -  0 / current                 ✅ 1 solution (start state)
-  0 →   1 →   -  0 / Become Vertical         ✅ Received 1, produced 1
-  0 →   0 →   -  0 / close hand for push     ❌ Received 1, produced 0 - FAILED!
-  0 →   0 ←   2  - / move to button          ⏸️ Waiting (2 solutions from backward)
2  - ←   2 →   -  2 / push button             ✅ Container generated 2 solutions
```

### Stage Types

| Stage Type     | Example                                         | Behavior                                 |
| -------------- | ----------------------------------------------- | ---------------------------------------- |
| **Generator**  | `CurrentState`, `GenerateGraspPose`             | Creates solutions, propagates both ways  |
| **Propagator** | `MoveTo`, `MoveRelative`, `ModifyPlanningScene` | Receives from one side, outputs to other |
| **Connector**  | `Connect`                                       | Plans paths between two endpoints        |

### Common Error Messages

| Error                          | Cause                             | Fix                                         |
| ------------------------------ | --------------------------------- | ------------------------------------------- |
| `Start state is out of bounds` | Joint at/beyond URDF limit        | Adjust SRDF values inside limits            |
| `X.XXXXX no IK found`          | `setMinSolutionDistance` too high | Reduce to 0.1 or less                       |
| `min_fraction not met`         | Cartesian path failed             | Increase distance or use sampling planner   |
| `Invalid goal state`           | Goal pose in collision            | Increase distance, check collision settings |
| `Deviation in joint X`         | Gripper state mismatch at Connect | Add gripper motion stage before Connect     |

---

## Octomap Save/Load Utility

A utility node was created to save and load octomaps for later use.

### Files Added

- `/home/rafilappy/ws_moveit_forked/src/custom_plan/src/octomap_saver.cpp`
- `/home/rafilappy/ws_moveit_forked/src/custom_plan/launch/octomap_saver.launch.py`

### Usage

```bash
# Run the node
ros2 launch custom_plan octomap_saver.launch.py

# Save current octomap
ros2 service call /octomap_saver/save_octomap std_srvs/srv/Trigger

# Load saved octomap
ros2 service call /octomap_saver/load_octomap std_srvs/srv/Trigger

# Clear octomap
ros2 service call /octomap_saver/clear_octomap std_srvs/srv/Trigger
```

### Parameters

| Parameter          | Default                | Description                     |
| ------------------ | ---------------------- | ------------------------------- |
| `save_directory`   | `/tmp/moveit_octomaps` | Directory to save/load octomaps |
| `default_filename` | `octomap`              | Base filename for saved files   |

Saved files use `.bt` format (octomap binary) and can be viewed with `octovis`.
