# SIGGRAPH_lousd

The coded snippet is designed to control a UR10 robot arm and 2F-140 robotics gripper digital twin attached with a cube that the user can manipulate. By transforming the cube, the robot arm will adjust its poses and align the orientation and position of its end effector to the cube.

## Getting started

- Go to `tasks/follow_target.py`, on line 55, change the `asset_path` to the path of the robot that you just created. By default it will use the robot stored in the Isaac Sim Content Library. 
- Remember to update the prim path and end effector path on line 61, 70, 72 in `tasks/follow_target.py if those are different.

## To Run

`path/to/isaac_sim/python.sh follow_target_example.py`

For example

`~/isaac_sim/python.sh follow_target_example.py`
