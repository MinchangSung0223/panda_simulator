# panda_controller_libfranka

```bash
   cd ~/catkin_ws/src; git clone https://github.com/justagist/franka_ros_interface.git;
   cd ~/catkin_ws/src; git clone https://github.com/ros-planning/moveit_tutorials.git -b melodic-devel;
   cd ~/catkin_ws/src; git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel;
   apt-get install -y ros-melodic-franka-msgs
   apt-get install -y ros-melodic-franka-hw
   apt-get install -y ros-melodic-franka-control
   
   
   
   # libfranka install
   git clone --recursive https://github.com/frankaemika/libfranka
   cd libfranka
   mkdir build
   cd build
   cmake ..
   make -j16
   
   # jsoncpp install
    git clone https://github.com/open-source-parsers/jsoncpp.git
    cd jsoncpp
    mkdir build
    cd build
    cmake ..
    make -j16
```
