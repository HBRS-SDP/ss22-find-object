# Connecting to Lucy
1. Connect to bit-bots@home wifi
2. 192.168.50.201/user/
3. 
```
export ROS_MASTER_URI=https://192.168.50.201:11311
```
4. 
```
ssh lucy@192.168.50.201
```


# Run launch files
1. 
```
roslaunch mdr_find_object_action find_object_client.launch
```
2.
```
rostopic pub /kcl_rosplan/action_dispatch rosplan_dispatch_msgs/ActionDispatch "action_id: 0
plan_id: 0
name: 'find_object'
parameters:
- {key: 'obj_name', value: 'Snacks'}
duration: 0.0
dispatch_time: 0.0" -1
```
3. 
```
roslaunch mdr_rosplan_interface rosplan.launch
```
4. devel branch
```
roslaunch mdr_find_object_action find_object.launch
```