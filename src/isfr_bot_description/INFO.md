# Gemaakt met:
```bash
ros2 pkg create --build-type ament_python isfr_bot_description --dependencies rclcpp std_msgs xacro
```

# Wat is dit?
Deze package beheert alles wat de bot/hardware beschrijft:
 - proto files
 - urdf files

## Folders
Eigen folders:
 - `./meshes/`: 3D data
 - `./protos/`: proto files = description voor webots
 - `./urdf/`: urdf files = description voor ros2

Alle andere folders zijn ROS-generated

## Files
nog niks