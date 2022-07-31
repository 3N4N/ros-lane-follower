## ROS Lane Follower

This is a lane-following vehicle built with ROS and OpenCV 4.2.

```
# if you want to visualize with Gazebo
roslaunch driving_track run.launch gui:=true
rosrun auto_drive automate_vehicle 0

# if you want to visualize with OpenCV image
roslaunch driving_track run.launch gui:=false
rosrun auto_drive automate_vehicle 1
```

### Demonstration

https://user-images.githubusercontent.com/32037751/182029113-64c2a53a-de12-4180-87d4-27cc1ae5b927.mp4
