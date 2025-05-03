# Robotics Project

## Run
First the package needs to be installed (like in assignment 2). 

- Terminal 1: ros2 launch project integration.launch name:=/rm0
- Terminal 2: ros2 launch project controller.launch name:=/rm0
- Terminal 3: pixi run coppelia
- In coppelia open the ./scenes/project.ttt scene

Once everything is running, a window with the live-feed of the camera should open.

## Information
According to the [manual](https://dl.djicdn.com/downloads/robomaster-s1/20200324/RoboMaster_S1_User_Manual_v1.8_EN.pdf) the pitch angle is in [-20,35] degrees. However, in the simulation the maximum pitch angle seems to be +30 degrees.

## TODO
- Play sound to simulate shooting
