# Gemaakt met:
```bash
ros2 pkg create --build-type ament_python isfr_bot_webots
```

# Wat is dit?
Deze package launcht webots en alle nodige controllers.
Er is ook een custom node die de odometrie van de diffdrive controller vervangt met perfecte odometry:
disable deze door `odom_gt:=false` aan de launch file toe te voegen.
<!-- TODO Die odom_gt en een fatsoenlijke launch file moet ik nog ff maken als ik refactor -->

## Folders
Eigen folders:
 - `./controllers/`: Bevat config files voor ros2 controllers (controllers gebruiken topics defined door de urdf)
 - `./worlds/`: Webots werelden (❗❗❗)
 - `./launch/`: Launcher voor basic webots-ros op te starten. De bringup launcher gebruikt deze ook
 <!-- TODO dit moet nog gebeuren in de refactor -->


## Files
 - `./controllers/ros2_control.yml`: Configureert de 2 controllers:
     - diffdrive = een controller om met 2 wielen te kunnen rijden
     - JointStateBroadcaster = een controller die ros de state van elke joint(motor / sensor) laat zien
 - `./isfr_bot_webots/ground_truth_odom_publisher.py`: Dit is een custom ros executable, die een custom ros node opstart om vanuit gps en imu data een "perfecte" ( = ground-truth) odometry te maken, om de ULTRA-slechte odom die generated is door de diffdrive controller te vervangen


## ❗❗❗ Om op te letten:
 - Als je via webots de world file wil aanpassen, zal webots de robot verwijderen als je saved. Kopieer dus altijd de  EXTERNPROTO en de robot node om die na het saven terug te zetten. (of haal die uit de laatste commit, als je niet de robot hebt aangepast en tegelijk de world aanpast zonder commit)
