# Steps to follow for real UR3e Arm

### Robot Preparation
- Make sure External Control URCaps is running
- UR Driver should say: `Robot connected to reverse interface. Ready to receive control commands.`
---
### ROS Preparation
```bash
cd ~/workspace
catkin build
source devel/setup.bash # Once per terminal instance
```
---
### Launch Sequence
```bash
# Initiate UR driver communication with real robot
roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.77.22 kinematics_config:=/home/user/catkin_ws/src/ur3e2_calib.yaml z_height:=0.77

# Start MoveIt for UR3e. It also launches Rviz
roslaunch ur3e_moveit_mrc ur3e_moveit.launch
```
