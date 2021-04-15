# dual-arm-workcell

I built this catkin workspace for simulating the dual-arm work-cell I am using for my robotic manipulation research. The workcell constitutes two UR3 robot arms equipped with 2F-85 robotiq grippers. The grippers have camera mounted on them manually. I carried out the intrinsic and extrinsic calibrations here. The workcell has a cage with multiple cameras, mainly to provide visual and point cloud data.

## The simulated workcell/robot
This is the robot in the simulation after building its xacro file that describes the workscell structure as well as the robots and the connected grippers.

![](./img/workcell.png)

Here is a closed look on the dual-arm robot
![](./img/robot.png)
## Purpose
This project is for porting my research studies to be done using ROS and MoveIt. It will also serve as my virtual lab for single arm and dual-arm manipulation planning. 

## Plan
I am planning to do the following with this workcell

1. Add gripper camera as well as camera to provide PCL data.
2. Reproduce tasks 
   1. Constrained motion planning.
   2. Graph-based motion planning.


## Acknowledgement
My knowledge to build such systems has been accumulated while working closely with my PhD direct supervisor and friend [Dr. Weiwei Wan](https://wanweiwei07.github.io/). It has been a very interesting journey full of exploration, hard work, and inspiration. 

