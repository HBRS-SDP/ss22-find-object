# ss22-find-object
## Table of Contents
1. [Summary](Summary)
2. [UML Diagram](UML_Diagram)
3. [Dependencies](Dependencies)
4. [Packages](Packages)
5. [Actions Used](Actions_Used)
6. [Acknowledgments](Acknowledgments)

## Summary

## UML_Diagram

## Dependencies
* mas_perception_libs
* mdr_move_base_action
* mdr_perceive_plane_action
* mdr_pickup_action
## Packages
* mas_knowledge_base - ontology
* mdr_navigation - navigating through the map
* mdr_perception - object recognition
* mdr_manipulation - pickup and place
* mdr_msgs - ROS messages 

## Actions_Used
1. 
2.
3.
4.

## Example usage
1. Run the rosplan interface: ``` roslaunch mdr_rosplan_interface rosplan.launch ```
2. Run the perceive plane action server: ``` roslaunch mdr_perceive_plane_action perceive_plane.launch cloud_topic:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points ```
3. Run the pick up action server: ``` roslaunch mas_hsr_pickup_action pickup_action.launch ```
4. Run the find object action server: ```roslaunch mdr_find_object_action find_object.launch```
5. Run the find object client: ``` roslaunch mdr_find_object_action find_object_client.launch ```
6. Run rviz and localize the robot
7. Publish the message (example):
```  
rostopic pub /kcl_rosplan/action_dispatch rosplan_dispatch_msgs/ActionDispatch "action_id: 0
plan_id: 0
name: 'find_object'
parameters:
- {key: 'obj_name', value: '055_baseball'}
duration: 0.0
dispatch_time: 0.0" -1

```

## Acknowledgments
