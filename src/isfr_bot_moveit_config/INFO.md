# Gemaakt met:
```bash
ros2 pkg create --build-type ament_python isfr_bot_moveit_config --dependencies moveit_ros_planning_interface rclcpp
```

# Wat is dit?
Deze package beheert de config van MoveIt.
Deze bestuurt de manipulator (robot arm)

## Folders
Eigen folders:
 - `./launch/`: launch script voor moveIt te starten met custom config/API
 - `./config/`: config files voor moveIt

Alle andere folders zijn ROS-generated

## Files
nog niks