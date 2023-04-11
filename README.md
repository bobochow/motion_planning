# motion planning
[![Maintenance](https://img.shields.io/badge/Maintained%3F-no-red.svg)](https://bitbucket.org/lbesson/ansi-colors)


# 1. Introduction
- This project implements several motion planning algorithm,including A*,ARA*,D*,JPS,Lazy Theta*,which were developed on the base of the Course [深蓝学院 浙大高飞主讲《移动机器人运动规划》](https://www.shenlanxueyuan.com/course/521).
- The note [ Motion Planning 学习笔记 by bo zhou](https://note.youdao.com/s/8DqZpzSO) can help you understand the core idea of these algorithm.
# 2. Quick start
1. Our software is developed and tested in Ubuntu 18.04, ROS Melodic.ROS can be installed here: [ROS Installation](http://wiki.ros.org/ROS/Installation).
2. Build on ROS
    - You can create an empty new workspace and clone this repository to your workspace:
    ```
    cd ~/your_catkin_ws/src
    git clone https://gitee.com/milab_402/motion_planning.git
    cd ..
    ```
    - Then, compile it.
    ```
    catkin_make
    ```
3. Run the Simulation.
    - Roslaunch demo
    ```
    source devel/setup.bash
    roslaunch grid_path_searcher demo.launch
    ```
    - Choose the "3D NAV Goal" rviz tool to set goal.

