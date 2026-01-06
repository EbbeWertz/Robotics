# Gemaakt met:
```bash
ros2 pkg create --build-type ament_python isfr_bot_bringup --dependencies rclcpp webots_ros2_driver
```

# Wat is dit?
Dit is basically de "main" package die alles launched.

## Folders
Eigen folders:
 - `./launch/`: Launch script om alle ros2 pacakges te starten
 - `./worlds/`: webots world file

Alle andere folders zijn ROS-generated

## Files
nog niks