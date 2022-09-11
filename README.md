# ss22-find-object
## Table of Contents
1. [Summary](Summary)
2. [UML Diagram](UML_Diagram)
3. [Dependencies](Dependencies)
4. [Packages](Packages)
5. [Actions Used](Actions_Used)
6. [Acknowledgments](Acknowledgments)

## Summary
* Given a user-specified choice(string) of object name, the software shall return all the default locations (strings) relative to the specified item based on the ontology structure.

![image](https://github.com/HBRS-SDP/ss22-find-object/blob/main/images/ontology.png)
![image](https://github.com/HBRS-SDP/ss22-find-object/blob/main/images/ontology_final.png)

* The interpreted the possible location of the object in the ontology is used along with the navigation goals.yml file to obtain the coordinates of the location which are then sent to move base server to move the robot.

* The robot shall navigate to the respective location and percives the plane using perceive plane action package to identify the user-specified object.

![image](https://github.com/HBRS-SDP/ss22-find-object/blob/main/images/final_demo_objects.png)

* If the object is not found the robot shall navigate to the next possible location to find the object.
* If the object is found then the robot shall pickup the user-specified object using pickup action code.
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
- {key: 'obj_name', value: 'baseball'}
duration: 0.0
dispatch_time: 0.0" -1
```

## Things to consider
1. You may need to restart services on the robot.
    * mas_hsr_pickup_action
    * mas_hsr_move_arm_action
    * mas_hsr_move_base_action
2.If rosplan interface gets stopped due to MongoDb store has been killed error, you might need to delete the files inside the mongoDB_store folder present in mas_knowledge_base/common and keep the folder empty before running the rosplan again.
3. Localize the robot properly before publishing the user specified object.
