# Gemaakt met:
```bash
ros2 pkg create --build-type ament_cmake isfr_bot_nav
```

# Wat is dit?
Deze package start de nodige nodes om te navigeren met nav2

## Te gebruiken bringup launcher om deze package te testen:
 - `isfr_bot_bringup/launch/simple_nav_setup.launch.py`


## Folders
Eigen folders:
 - `./launch/`: Launch file (launcht enkel de navigatie nodes. Rviz en webots worden gelauncht door bringup)
 - `./config/`: visualisatie config voor rviz + nav2 config voor navigatie

## ❗❗❗ Om op te letten:
Als je de webots world veranderd, maak dan een nieuwe map aan: zie de mapping package.