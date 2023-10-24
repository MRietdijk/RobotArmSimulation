# Installatie handleiding Robotarm-simuatie

## Benodigdheden
 - ROS humble Hawksbill
 - RViz
 - Tf2

## Installeren

1. Maak een Ros-workspace aan door het volgende commando uit te voeren.
   
   `mkdir -p ws/src`

2. Ga naar de workspace.
   
   `cd ws/src`

3. Clone de repo
   
   `git clone https://github.com/MRietdijk/RobotArmSimulation.git .`

4. Ga naar de root van je workspace.
   
   `cd ..`

5. Build de applicatie.
   
   `colcon build`

### Runnen
6. Source de applicatie.
   
   `source ./install/setup.bash`

7. Voer het volgende commando uit.
   
   `ros2 launch simulation launch_file.launch.py`

8. Open een nieuwe terminal.
   
   (Op Ubuntu 22.04: `ctrl + alt + t`)

9.  Ga naar je workspace.
    
    `cd ws`

10. Source de applicatie
    
    `source ./install/setup.bash`

11. Voer het volgende commando uit.
    
    `ros2 run console sequence`