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
 - De installed packages krijg je door de commands in `./install-packages.sh`
 - Als installed packages custom config of scripts hebben, dan is er een eigen pacakge in `/src/` als een soort wrapper of launcher.
 - Elke pacakges in `./src/` heeft een `INFO.md` als guide voor die pacakage.

 # Meer info:
  - `./PACKAGES.md` bevat een lijst van alle relevante pacakges (eigen packages in `./src/` maar ook installed packages)
  - Elke package heeft een `INFO.md` waar informatie voer die pacakge in staat 
  - `./ROS_INFO` beschrijft per installed package alle (relevante):
    - Nodes
    - Services
    - Topics
    - Actions
  - De Nodes, services, ... van eigen packages in `./src/` staan ook in elke `INFO.md` file (vanaf dat daar code in die pacakges zit)


# Wat doen?

## 1Ste keer:
 - run `bash ./install-pacakges.sh`
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
