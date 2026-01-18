# Gemaakt met:
```bash
ros2 pkg create --build-type ament_cmake isfr_bot_mapping
```

# Wat is dit?
Deze package start de nodige nodes om slam toolbox te gebruiken voor mapping

## Te gebruiken bringup launcher om deze package te testen:
 - `isfr_bot_bringup/launch/mapping_setup.launch.py`

## Relevante scripts voor deze package
Als je mapping gebruikt moet je de robot manueel besturen.
--> Gebruik dus in 2de terminal:
```bash
source scripts/setup_norebuild.sh
bash scripts/keyboard_controller.sh
```
Als de map vol (genoeg) is kan je de map saven naar een pgm en yaml file:
--> Gebruik dus in 3de terminal:
```bash
source scripts/setup_norebuild.sh
bash scripts/save_map.sh
```
Dan krijg je een pmg file en yaml file, die je kan moven naar de `config/map` folder van de nav package

## Folders
Eigen folders:
 - `./launch/`: Launch file (launcht enkel de mapping nodes. Rviz en webots worden gelauncht door bringup)
 - `./config/`: visualisatie config voor rviz + slam_toolbox config voor mapping