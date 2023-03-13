# FAEP

**FAEP** is a nove fast autonomous exploration planner based on the framework of [FUEL](https://ieeexplore.ieee.org/abstract/document/9324988). It contains a comprehensive exploration sequence generation method for global tour planning, which not only considers the flight-level factors but also innovatively considers the frontier-level factors to reduce the back-and-forth maneuvers. In addition, an adaptive yaw planning strategy is designed to achieve efficient exploration by yaw change during flight.  
Our method is demonstrated to reduce the flight time and flight distance by more than 20% compared with the state-of-the-art approache FUEL.

<p align="center">
  <img src="files/1.gif" width = "400" height = "225"/>
  <img src="files/2.gif" width = "400" height = "225"/>
</p>

Complete videos: [video1](https://www.youtube.com/watch?v=0Y671mEwJ_A).

Please cite our paper if you use this project in your research:
- [Autonomous Exploration Method for Fast Unknown Environment Mapping by Using UAV Equipped with Limited FOV Sensor](https://arxiv.org/abs/2302.02293), Yinghao Zhao, Li Yan, Hong Xie, Jicheng Dai, Pengcheng Wei.

```
@article{zhao2023autonomous,
  title={Autonomous Exploration Method for Fast Unknown Environment Mapping by Using UAV Equipped with Limited FOV Sensor},
  author={Zhao, Yinghao and Yan, Li and Xie, Hong and Dai, Jicheng and Wei, Pengcheng},
  journal={arXiv preprint arXiv:2302.02293},
  year={2023}
}
```

Please kindly star :star: this project if it helps you. We take great efforts to develope and maintain it :grin::grin:.


## Quick Start

This project is mostly based on [FUEL](https://github.com/HKUST-Aerial-Robotics/FUEL). Therefore, the configuration of this method is the same as FUEL.
It has been tested on Ubuntu 16.04(ROS Kinetic) and 18.04(ROS Melodic). Take Ubuntu 18.04 as an example, run the following commands to setup:

```
  sudo apt-get install libarmadillo-dev ros-melodic-nlopt libdw-dev
```

To simulate the depth camera, we use a simulator based on CUDA Toolkit. Please install it first following the [instruction of CUDA](https://developer.nvidia.com/zh-cn/cuda-toolkit). 

After successful installation, in the **local_sensing** package in **uav_simulator**, remember to change the 'arch' and 'code' flags in CMakelist.txt according to your graphics card devices. You can check the right code [here](https://github.com/tpruvot/ccminer/wiki/Compatibility). For example:

```
  set(CUDA_NVCC_FLAGS 
    -gencode arch=compute_61,code=sm_61;
  ) 
```

Finally, clone and compile our package:

```
  cd ${YOUR_WORKSPACE_PATH}/src
  git clone https://github.com/Zyhlibrary/FAEP.git
  cd ../ 
  catkin_make
```

After compilation you can start the visualization by: 

```
  source devel/setup.bash && roslaunch exploration_manager rviz.launch
```
and start a simulation (run in a new terminals): 
```
  source devel/setup.bash && roslaunch exploration_manager exploration.launch
```
You will find a cluttered scene to be explored (20m x 12m x 2m) and the drone in ```Rviz```. You can trigger the exploration to start by the ```2D Nav Goal``` tool. A sample simulation is shown in the figure. The unknown obstacles are shown in grey, while the frontiers are shown as colorful voxels. The planned and executed trajectories are also displayed.


## Acknowledgements
  We use **NLopt** for non-linear optimization and use **LKH** for travelling salesman problem.
