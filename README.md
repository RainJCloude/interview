
After building the package, type:
```
ros2 launch points_from_1_to_7 frames.launch.py 
```
to launch the frame publisher and open rviz

The rotating fram will move with a default linear velocity along the z axis and will rotate about the z axis

You can select these velocity passing the parameters:
```
ros2 launch points_from_1_to_7 frames.launch.py ang_vel:=desiredAng_vel lin_vel:=desiredLinVel
```
The launch file will also launch the im_pub node which will publish an image on the /camera/Image topic along with camera_info informations

In another terminal, type:
```
ros2 run points_from_1_to_7 transform
```
to see the transformation between the camera_optical_frame and the rotating_frame.

The point 7 has been implemented through the drawer node, which has been launched in the same launch file


Typing:
```
ros2 run point_8 ray_tracer 
```

will open a colored image. Clicking on the image will print the (x, y) coordinates in the world frame (assuming Z=1) in the terminal, while simultaneously publishing these coordinates as a geometry_msgs::msg::Pose message on the /ray topic.