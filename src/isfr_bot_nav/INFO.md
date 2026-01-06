# Gemaakt met:
```bash
ros2 pkg create --build-type ament_python isfr_bot_nav --dependencies nav2_bringup rclcpp
```

# Wat is dit?
Deze package beheert de config van navigation2.
Deze zorgt voor navigatie van de bot in de world

## Folders
Eigen folders:
 - `./launch/`: launch script voor nav2 te starten met custom config

Ros folders die we gebruiken:
 - `./resource/`: config files voor nav2 (nav2 verwacht deze in resource)

Alle andere folders zijn ROS-generated

## Files
nog niks