# Package Interfaces Overview
(generated door chatGPT)

## Installed System Packages

### ros-jazzy-webots-ros2
**Role:** ROS 2 ↔ Webots integration layer.

- **Nodes:**
  - `webots_ros2_driver`
- **Topics:**
  - `/clock`
  - `/tf`
  - `/tf_static`
  - Sensor topics bridged from Webots (e.g. `/scan`, `/camera/image_raw`)
- **Services:**
  - `/reset_simulation`
  - `/pause_simulation`
  - `/resume_simulation`
- **Actions:** None

---

### ros-jazzy-turtlebot3-description
**Role:** Robot description resources for TurtleBot3.

- **Nodes:** None
- **Topics:** None
- **Services:** None
- **Actions:** None

---

### ros-jazzy-turtlebot3-navigation2
**Role:** Navigation2 configuration for TurtleBot3.

- **Nodes:** (instantiated via Nav2)
- **Topics:**
  - `/cmd_vel`
  - `/odom`
  - `/map`
- **Services:** Provided by Nav2 servers
- **Actions:**
  - `/navigate_to_pose`

---

### ros-jazzy-turtlebot3-bringup
**Role:** Standard TurtleBot3 system bringup.

- **Nodes:**
  - `robot_state_publisher`
  - Sensor drivers
- **Topics:**
  - `/joint_states`
  - `/tf`
- **Services:** Node lifecycle services
- **Actions:** None

---

### ros-jazzy-open-manipulator-description
**Role:** Robot description resources for OpenManipulator.

- **Nodes:** None
- **Topics:** None
- **Services:** None
- **Actions:** None

---

### ros-jazzy-open-manipulator-moveit-config
**Role:** MoveIt configuration for OpenManipulator.

- **Nodes:**
  - `move_group`
- **Topics:**
  - `/joint_states`
  - `/planning_scene`
- **Services:**
  - `/get_planning_scene`
- **Actions:**
  - `/move_action`

---

### ros-jazzy-xacro
**Role:** URDF macro processing tool.

- **Nodes:** None
- **Topics:** None
- **Services:** None
- **Actions:** None

---

### ros-jazzy-nav2-bringup
**Role:** Core Navigation2 bringup and lifecycle management.

- **Nodes:**
  - `controller_server`
  - `planner_server`
  - `bt_navigator`
  - `amcl`
- **Topics:**
  - `/cmd_vel`
  - `/goal_pose`
  - `/map`
- **Services:**
  - `/clear_costmaps`
  - `/set_initial_pose`
- **Actions:**
  - `/navigate_to_pose`

---

## Self-Created Workspace Packages

### isfr_bot_description
**Role:** Defines the full robot model for ROS 2 and Webots.

- **Nodes:** None
- **Topics:** None
- **Services:** None
- **Actions:** None

---

### isfr_bot_bringup
**Role:** Launches and coordinates the robot in simulation.

- **Nodes:**
  - `webots_ros2_driver`
  - `robot_state_publisher`
- **Topics:**
  - `/clock`
  - `/tf`
  - `/joint_states`
- **Services:**
  - Lifecycle services for launched nodes
- **Actions:** None

---

### isfr_bot_moveit_config
**Role:** Motion planning for the robot manipulator.

- **Nodes:**
  - `move_group`
- **Topics:**
  - `/joint_states`
  - `/trajectory_execution_event`
- **Services:**
  - `/get_planning_scene`
- **Actions:**
  - `/move_action`
  - `/execute_trajectory`

---

### isfr_bot_nav
**Role:** Autonomous navigation for the mobile base.

- **Nodes:**
  - `planner_server`
  - `controller_server`
  - `bt_navigator`
  - `amcl`
- **Topics:**
  - `/cmd_vel`
  - `/map`
  - `/odom`
  - `/scan`
- **Services:**
  - `/clear_costmaps`
- **Actions:**
  - `/navigate_to_pose`

---

### isfr_bot_utils
**Role:** Development and model-conversion utilities.

- **Nodes:** None
- **Topics:** None
- **Services:** None
- **Actions:** None
