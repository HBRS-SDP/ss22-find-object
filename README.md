# ss22-find-object
For the "General Solution to Find Object" project, done for the course of SDP-SS22, we were required to program the Human Support HSR so as to work like a human being for finding objects. For example, if a human being was told to get a spoon, their first response would be to go to the kitchen and then dining room, then living room, etc. The same logic has been implemented, where the HSR has to go through the ontology to get the default location of the user-specified item, go to the default location and perceive for the item. If the item is not found in the  default location, then it has to go to all the other possible locations to perceive and pick up the object. 

To get started with building and learning the domestic packages, the [mas_tutorials](https://github.com/b-it-bots/mas_tutorials#mas_tutorials) page has been helpful.

The strategy implemented has been explained below.

## Table of Contents
1. [Summary](https://github.com/HBRS-SDP/ss22-find-object#summary)
2. [UML Diagram](https://github.com/HBRS-SDP/ss22-find-object#uml_diagram)
3. [Dependencies](https://github.com/HBRS-SDP/ss22-find-object#dependencies)
4. [Packages](https://github.com/HBRS-SDP/ss22-find-object#packages)
5. [Example Usage](https://github.com/HBRS-SDP/ss22-find-object#example-usage)
6. [Things to Consider](https://github.com/HBRS-SDP/ss22-find-object#things-to-consider)
7. [Acknowledgments](https://github.com/HBRS-SDP/ss22-find-object#acknowledgments)

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
![image](https://github.com/HBRS-SDP/ss22-find-object/blob/main/images/output.svg)
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
* mdr_hri - sound vocalization, graphical user interfaces

## Example usage
1. Run the rosplan interface: ``` roslaunch mdr_rosplan_interface rosplan.launch ```
2. Run the perceive plane action server: ``` roslaunch mdr_perceive_plane_action perceive_plane.launch cloud_topic:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points ```
3. Run the pick up action server: ``` roslaunch mas_hsr_pickup_action pickup_action.launch ```
4. Run the find object action server: ```roslaunch mdr_find_object_action find_object.launch```
5. Run the find object client: ``` roslaunch mdr_find_object_action find_object_client.launch ```
6. Run rviz and localize the robot
7. Publish the message (example object- baseball):
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

## Acknowledgments
This software would be impossible without

* the many generations of b-it-bots@Home members. A list of contributors can be found [here](https://github.com/b-it-bots/mas_domestic_robotics/graphs/contributors)
* the MAS staff and professors who have provided their advice and support.
