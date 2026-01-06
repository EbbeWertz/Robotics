# Gemaakt met:
```bash
ros2 pkg create --build-type ament_python isfr_bot_utils
```

# Wat is dit?
Hier kunnen helper scripts en tijdelijke of extra test shit in

## Folders
Eigen folders:
 - `./scripts/`: util scripts

Alle andere folders zijn ROS-generated

## Files
 - `./scripts/xacro_to_urdf.sh`: Deze leest de xacro urdf descriptions van de turtlebot(waffle) en de openmanipulator(x) uit de installed description packages, en maakt pure (embedded, webots-friendly) urdf files in de `isfr_bot_description/urdf/` folder\
 !! RUN met een terminal IN de scripts folder
  - `./scripts/urdf_to_proto.sh`: Deze leest de generated urdf files uit de `isfr_bot_description` pacakge folder en generate proto files voor webots in de `isfr_bot_description/protos/` folder\
 !! RUN met een terminal IN de scripts folder