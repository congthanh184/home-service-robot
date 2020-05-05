# Home Service Robot

Simulate a service robot in a static environment. The robot will navigate to pick up and deliver a virtual object. The object is model with marker in rviz.                                                                                                                                                                       
Project utilises ROS, gazebo and rviz to simulate environment and robot with lidar and rgb-d camera. It also includes acml, teleop, gmapping and actionlib packages to build map, perform localization and drive robot to target.

## Map Setup with ACML and Teleop packages

Run `scripts/test_slam.sh`
Manually move the robot to explore the map.
Then, open the `rtabmap.db` by using `rtabmap-databaseViewer ~/.ros/rtabmap.db`
Select `File > Export optimized 2D map` to build the 2D map.
Alternatively, run 

```
rosrun rtabmap_ros rtabmap _database_path:=/home/.ros/rtabmap.db
rosrun map_server map_saver map:=proj_map
rosservice call /publish_map 1 1 0
```

## Test run

To test navigation, including cost_map, run `scripts/test_navigation.sh`

To test move to target pose alone, run `scripts/pick_objects.sh`

To test display markers as virtual objects, run `scripts/add_markers`

## Put together

Run `scripts/home_service.sh`

**Note**:

- On ubuntu, use `gnome-terminal -- <command>` to open a new terminal for the commands
- On Kubuntu, use `xterm`
