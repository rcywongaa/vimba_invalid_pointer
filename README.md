All commands to be run in the base `vimba_invalid_pointer` directory

# Setup
```
sudo apt install python-catkin-tools
catkin config -s catkin_ws
```

# Build
```
catkin build
source devel/setup.bash
source devel/setup.zsh # Depending on bash / zsh shell
```

# Launch
```
roslaunch camera main.launch
```

# Reproduce Bug
1. Edit `catkin_ws/camera/src/node.cpp`
1. Uncomment lines
   ```
       //ros::init(argc, argv, "camera");
       //ros::NodeHandle nh;
   ```
1. Rebuild and launch
