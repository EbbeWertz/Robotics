# Wat is er allemaal?

Deze folder is de ros2 **workspace**

Hier zijn build time files:
 - `./build/` (gitignored)
 - `./install/` (gitignored)
 - `./log/` (gitignored)

En er zijn ook ros pacakges:
 - Installed pacakges (die je niet ziet)
 - Eigen packages: dit zijn de folders in `./src/`

Over de packages:
 - De installed packages krijg je door de commands in `./scripts/install-packages.sh`
 - Als installed packages custom config of scripts hebben, dan is er een eigen pacakge in `/src/` als een soort wrapper of launcher.
 - Elke pacakges in `./src/` heeft een `INFO.md` als guide voor die pacakage.

 # Meer info:
  - Elke package heeft een `INFO.md` waar informatie voer die pacakge in staat 
  - `./ROS_INFO` beschrijft per installed package alle (relevante):
    - Nodes
    - Services
    - Topics
    - Actions
  - De Nodes, services, ... van eigen packages in `./src/` staan ook in elke `INFO.md` file (vanaf dat daar code in die pacakges zit)


# Wat doen?

## 1Ste keer:
 - run `bash ./scripts/install_pacakges.sh`
 - check of alle pacakges er zijn met:
   - run `ros2 pkg list | grep turtlebot3` en je moet zien: turtlebot3_*
      - *_bringup
      - *_description
      - *_msgs
      - *_navigation2
      - *_node
   - `ros2 pkg list | grep open_manipulator`
      - *_description
      - *_moveit_config
   - `ros2 pkg list | grep webots_ros2`
      - gwn webots_ros2 zonder iets erachter
      - *_control
      - *_driver
      - *_importer
      - *_msgs
      - *_turtlebot3
      - andere robots die niet boeien
 - maak `WEBOTS_HOME` variable:
   - bvb: `export WEBOTS_HOME=/mnt/c/Program\ Files/Webots`
   - of: `export WEBOTS_HOME=/mnt/c/Users/houwe/AppData/Local/Programs/Webots`

## Runnen:
 ### Launcher:
  - run `bash ./scripts/setup_and_build.sh`
  - run `ros2 launch isfr_bot_bringup <launch file>.py <parameters>`
 ### Andere nodes:
  - Open nieuwe terminal
  - run `bash ./scripts/setup_norebuild.sh`
  - run whatever package je wil runnen

## Scripts in de scripts folder:
 - `install_packages.sh`: installeert alle nodige ros packages
 - `keyboard_controller.sh`: start de teleop-twist keyboard om de robot manueel te besturen
 - `save_map.sh`: Als je de robot start in SLAM mode, kan je hiermee de map saven in een `*.pgm` en `*.yaml` file
 - `setup_and_build`: sources de ros2 jazzy env, Verwijdert build files, rebuild de project, source de build env en kill oude ros en webots instances --> gebruik elke keer als de project is veranderd
 - `setup_norebuild`: sources de ros2 jazzy env en de build env --> gebruik als je een 2de terminal voor ros wil gebruiken als ene al bezig is met de launch te runnen. ❗❗Run dit NA dat de setup_and_build in terminal 1 KLAAR is