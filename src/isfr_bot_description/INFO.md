# Gemaakt met:
```bash
ros2 pkg create --build-type ament_cmake isfr_bot_description
```

# Wat is dit?
Dit is de beschrijving van de bot, voor webots en voor ros2
(`<bot_name>_desscription` is een standaard naam om te gebruiken)

## Folders
Eigen folders:
 - `./urdf/`: URDF files voor ROS
 - `./protos/`: Proto files voor webots
 - `./scripts/`: scripts om urdf proto files te generaten (❗❗❗)


## Files
 - `./urdf/IsfrFullRobot`: Deze urdf wordt gelezen door de ros2-webots driver/controller. Allebei de robots kunnen hierin beschreven worden, maar joints die hier niet in staan worden automatisch gepublished.
 - `./protos/*.proto`: TurtleBot3Waffle = de base, (de arm staat er nu nog niet in), IsfrFullRobot is de combinatie van die 2, zodat er 1 proto is voor de volledige bot
 - `./scripts/xacro_to_urdf.sh`: Deze zoekt de (bestaande) xacro files van de turtlebot3_waffle en openmenipulator_x, en convert deze naar pure urdf files (Deze xacros waren gemaakt voor Gazebo, dus de urdfs zijn NIET voor ROS/webots gebruik) (❗❗❗)
 - `./scripts/urdf_to_proto.sh`: Deze neemt de generated (Gazebo-gebaseerde) urdf files en convert die naar Webots Proto files (gebruikt de urdf files NOOIT voor iets anders dan dit.)

## ❗❗❗ Om op te letten:
 - Als je een nieuwe folder maakt, add die dan `install(DIRECTORY <hier>` in de cmakelists file.
 - Als je de scripts gebruikt, laat de naam RAW om niet pergongelijk de "goede" protos te vervangen
     --> De manueel edited protos en urdfs zijn VEEL anders dan de generated
     --> de generated urdf moet ook enkel gebruikt worden om de proto file te generate. Gebruik NOOIT de generated urdf in ros
