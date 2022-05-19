# Steps to run rosdep
Way-01
Better use wstool instead
```
wstool init
wstool merge mas_domestic_robotics/mas-domestic-devel.rosinstall
wstool update           #skip the already cloned ones
```
You can then build
```
catkin build mdr_find_object_action
```

To run rviz locally
```
export ROS_MASTER_URI=http://192.168.50.201:11311
```

Way-02
The longer way:
- https://github.com/b-it-bots/mas_perception_libs, branch noetic
- https://github.com/b-it-bots/mas_perception_msgs, branch noetic
- https://github.com/b-it-bots/ros_dmp, branch master
- https://github.com/b-it-bots/mas_knowledge_base, branch devel
    - https://github.com/b-it-bots/mongodb_store, branch noetic-devel
    - pip install rdflib
    - pip install termcolor
    - git@github.com:b-it-bots/ROSPlan.git, branch master
    
