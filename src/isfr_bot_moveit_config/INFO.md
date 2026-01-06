# Gemaakt met:
package gemaakt:
```bash
ros2 pkg create --build-type ament_python isfr_bot_moveit_config --dependencies moveit_ros_planning_interface rclcpp
```
default configs gekopieeerd uit de open_manipulator_moveit_config install package:
```bash
cp -r $(ros2 pkg prefix open_manipulator_moveit_config)/share/open_manipulator_moveit_config/* ./isfr_bot_moveit_config/
# called vanuit de src root
```


# Wat is dit?
Deze package beheert de config van MoveIt.
Deze bestuurt de manipulator (robot arm)

## Folders
Eigen folders (gekopieerd uit de open_manipulator_moveit_config install package ):
 - `./launch/`: launch script voor moveIt te starten met custom config/API
 - `./config/`: config files voor moveIt
 - `./environment/`: (niet relevant)

Alle andere folders zijn ROS-generated

## Files
 - Geen idee, moet ik nog uitzoeken.
 - Er zijn wel files voor meerdere robots. Degenen met openmanipulator_x zijn voor de arm die wij gebruiken
 - `./launch/moveIt_launch.py` is onze custom launch file