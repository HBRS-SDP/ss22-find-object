# Connecting to Lucy
1. Connect to bit-bots@home wifi
2. 192.168.50.201/user/
3. 
```
export ROS_MASTER_URI=http://192.168.50.201:11311
```
4. 
```
ssh lucy@192.168.50.201
```


# Run launch files
1. 
```
roslaunch mdr_rosplan_interface rosplan.launch
```
2. devel branch
```
roslaunch mdr_find_object_action find_object.launch
```
3. 
```
roslaunch mdr_find_object_action find_object_client.launch
```
4.
```
rostopic pub /kcl_rosplan/action_dispatch rosplan_dispatch_msgs/ActionDispatch "action_id: 0
plan_id: 0
name: 'find_object'
parameters:
- {key: 'obj_name', value: 'Fruit'}
duration: 0.0
dispatch_time: 0.0" -1
```

5. roslaunch mdr_perceive_plane_action perceive_plane.launch cloud_topic:=/hsrb/head_rgbd_sensor/depth_registered/rectified_points
