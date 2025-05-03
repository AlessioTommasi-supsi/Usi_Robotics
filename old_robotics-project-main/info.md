# How to control the RoboMaster S1 in coppelia using ROS2?

Instead of starting the communication node with `ros2 launch robomaster_ros ep.launch name:=/rm0` as in the assignment 2, we use `ros2 launch robomaster_ros s1.launch name:=/rm0`. I suspect this node does not have the distance sensors enabled.

The `robomaster_ros` package contains the implementation of this communication node. Note that the turret is called gimbal. In particular, the [code for the gimbal module](https://github.com/EliaCereda/robomaster_ros/blob/main/robomaster_ros/robomaster_ros/modules/gimbal.py) is relevant.

The relevant topics are:
- `/rm0/cmd_gimbal` with parameters `yaw_speed` and `pitch_speed`
- `/rm0/gimbal/mode` with integer parameter `mode` (set to 0 to make the gimbal control independent of the chasis)
- `/rm0/blaster_led` with integer parameter `brightness` (this is kind of useless since it is an LED and not a laser pointer)
- `/rm0/joint_states` to retrieve the states of the `gimbal_joint` and `blaster_joint` joints

# How to control RoboMaster S1 in coppelia using lua?
Consult the [robomaster_sim](https://github.com/jeguzzi/robomaster_sim/blob/main/lua_api.md) package for more information.

```lua 
-- Turn turret 90 degrees to the left and 45 degrees up

simRobomaster = loadPlugin 'simRobomaster';

local yaw = 3.14/2
local pitch = 3.14/4
local yaw_speed = 1.0
local pitch_speed = 1.0

simRobomaster.move_gimbal(0, yaw, pitch, yaw_speed, pitch_speed, "chassis", "fixed")
```
