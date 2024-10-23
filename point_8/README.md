
After building the package, type:
```
ros2 launch interview_homework frames.launch.py 
```
to launch the frame publisher and open rviz

The rotating fram will move with a default linear velocity along the z axis and will rotate about the z axis

You can select these velocity passing the parameters:
```
ros2 launch interview_homework frames.launch.py ang_vel:=desiredAng_vel lin_vel:=desiredLinVel
```
The launch file will also launch the im_pub node which will publish an image on the /camera/Image topic along with camera_info informations

In another terminal, type:
```
ros2 run interview_homework frame_transf
```
to see the transformation between the camera_optical_frame and the rotating_frame.

The point 7 has been implemented through the drawer node:
Make sure that the launch file has been executed to ensure that im_pub is running
```
ros2 run interview_homework drawer
```