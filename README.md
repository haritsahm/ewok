# Trajectory Replanning Based on RRT* and B-Spline 
---
See the original work here:

[Ewok: Real-Time Trajectory Replanning for MAVs using Uniform B-splines and 3D Circular Buffer](https://github.com/VladyslavUsenko/ewok)
---
## This is part of my undergraduate capstone project.
[![Animated Gif](https://media.giphy.com/media/Z9W9MSOHUSJKm7YrMZ/giphy-downsized-large.gif)](https://www.youtube.com/watch?v=qD0lT9ndMEY)

### 1. Related Papers
* **Real-Time Trajectory Replanning for MAVs using Uniform B-splines and 3D Circular Buffer**, V. Usenko, L. von Stumberg, A. Pangercic, D. Cremers, In 2017 International Conference on Intelligent Robots and Systems (IROS) [[DOI:10.1109/IROS.2017.8202160]](https://doi.org/10.1109/IROS.2017.8202160) [[arXiv:1703.01416]](https://arxiv.org/abs/1703.01416).
* **Sampling-based Algorithms for Optimal Motion Planning**, Sertac Karaman and Emilio Frazzoli [[arXiv:1105.1186]](https://arxiv.org/abs/1105.1186)
* **Informed RRT\*: Optimal Sampling-based Path Planning Focused via Direct Sampling of an Admissible Ellipsoidal Heuristic**, Gammell, Jonathan D. and Srinivasa, Siddhartha S. and Barfoot, Timothy D., In 2014 IEEE/RSJ International Conference on Intelligent Robots and Systems [[DOI:10.1109/IROS.2014.6942976]](https://doi.org/10.1109/IROS.2014.6942976) [[arXiv:1404.2334]](https://arxiv.org/abs/1404.2334)

### 2. Installation

The system has been tested with Ubuntu 18.04 (ROS Melodic).

Follow the tutorials to [install ROS Melodic](http://wiki.ros.org/ROS/Installation) and to [set up catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Install additional dependencies:
```
sudo apt-get install libeigen3-dev libsuitesparse-dev protobuf-compiler libnlopt-dev ros-melodic-octomap ros-melodic-octomap-ros ros-melodic-sophus libatlas-base-dev python-matplotlib python-numpy
```

Navigate to the source folder of your catkin workspace, download and build the software:
```
cd ~/catkin_ws/src
git clone https://github.com/VladyslavUsenko/ewok.git
cd ewok
git submodule init
git submodule update

cd ../../
catkin_make
```
Some of the provided examples (trajectory_replanning_big_forest.launch) depend on the forest_gen dataset. With the above commands it should already be downloaded as a git submodule. Please check https://github.com/ethz-asl/forest_gen for more information about the dataset.

### 3. Simulation example
In separate terminal windows:

Start roscore:
```
roscore
```

Start visualization:
```
roscd ewok_simulation/rviz/
rviz -d simulation_rrt.rviz
```

Launch the system and simulator:
```
roslaunch ewok_simulation trajectory_replanning_simulation_rrt.launch 
```

by default, the step_size = 0.25 m and flat height is enabled. You can change the parameter in launch file or adding the parameter when launching:
```
roslaunch ewok_simulation trajectory_replanning_simulation_rrt.launch step_size:=1.25 flat_height:=False
```

Now you should be able to see Rviz visualization of the system running in simulator. Due to performance reasons GUI of the Gazebo simulator is disabled by default.
To enable it, change the following parameter in `ewok_simulation/launch/trajectory_replanning_simulation.launch`:
```
<arg name="gui" default="true"/>
```

### Testing With Single Obstacle

[![](https://i.imgur.com/NNbvdIh.png)](https://www.youtube.com/playlist?list=PLXAepXpbdn-N7D-bDPp8fLf5Re6JKAj59)

Start visualization:
```
roscd ewok_simulation/rviz/
rviz -d simulation_single.rviz
```
Launch the system:
```
roslaunch ewok_optimization rrt_simulation_single.launch step_size:=1.25 flat_height:=False num_iter:=1500
```

## License
This project is licensed under the GNU Lesser General Public License Version 3 (LGPLv3). 
Ewok is licensed under the GNU Lesser General Public License Version 3 (LGPLv3). 
