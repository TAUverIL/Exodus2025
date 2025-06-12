Spawning the rover and running control nodes:
* Running launch file and spawning rover:  ```ros2 launch nav2_stanley ack_bringup.launch.py```
* Running initial control node:  ```ros2 run gazebo_ackermann_control control_node```
* Teleop command: ```ros2 run teleop_twist_keyboard teleop_twist_keyboard```

Creating SDF:
* Xacro -> URDF conversion:  ```xacro ack_rover.xacro.urdf > ack_rover.urdf```
* URDF -> SDF conversion:  ```ign sdf -p ack_rover.urdf > ack_rover.sdf```

ROS2 topics:
* List current topics: ```ros2 topic list```
* Echo to screen: ```ros2 topic echo /topic_name```

IGN topics:
* List ignition topics: ```ign topic -l```
* Echo to screen: ```ign -e -t /topic_name```

Check transforms:
* Create TF tree (saves PDF in current location): ```ros2 run tf2_tools view_frames```
* Verify transform exists between two TFs (for example, using map and odom TFs): ```ros2 run tf2_ros tf2_echo map odom```

General:
* Format for package creation with NAV2:
  ```ros2 pkg create <package_name> --build-type ament_python --dependencies rclpy geometry_msgs rviz2 robot_localization ros_gs_bridge ros_gz_sim nav2_bringup nav2_map_server nav2_lifecycle_manager tf2_ros```
* Make python file executable: ```chmod +x <filename>.py```

File locations:
* All packages installed with sudo apt install: ```/opt/ros/humble/lib```
* Package contents: ```/opt/ros/humble/share```
* Gazebo worlds: ```/usr/share/ignition/ignition-gazebo6/worlds/```

Quarternions:
This table shows quaternion values for planar yaw orientations around the Z-axis, in 45° increments (counter-clockwise rotation from +X).
| Angle (°) | Yaw (rad) | Quaternion (x, y, z, w)               |
|-----------|-----------|---------------------------------------|
| 0°        | 0         | (0.0, 0.0, 0.0000, 1.0000)            |
| 45°       | π/4       | (0.0, 0.0, 0.3827, 0.9239)            |
| 90°       | π/2       | (0.0, 0.0, 0.7071, 0.7071)            |
| 135°      | 3π/4      | (0.0, 0.0, 0.9239, 0.3827)            |
| 180°      | π         | (0.0, 0.0, 1.0000, 0.0000)            |
| 225°      | 5π/4      | (0.0, 0.0, 0.9239, -0.3827)           |
| 270°      | 3π/2      | (0.0, 0.0, 0.7071, -0.7071)           |
| 315°      | 7π/4      | (0.0, 0.0, 0.3827, -0.9239)           |

Waypoint follower node:
```ros2 run follow_waypoints follow_waypoints```

Publish list of waypoints when using waypoint follower node:
```
ros2 topic pub /waypoints nav_msgs/Path "{
    header: { frame_id: 'map' },
    poses: [
         { header: { frame_id: 'map' }, pose: { position: { x: 12.553, y: -5.205, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } },
         { header: { frame_id: 'map' }, pose: { position: { x: 16.795, y: -9.447, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } },
         { header: { frame_id: 'map' }, pose: { position: { x: 17.243, y: -16.243, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } },
         { header: { frame_id: 'map' }, pose: { position: { x: 9.447,  y: -16.795, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } },
         { header: { frame_id: 'map' }, pose: { position: { x: 5.205,  y: -12.553, z: 0.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }
    ]
}" --once
```

ROS2 command for single motor:
* Name of package: ```cubemars_test```
* Single motor launch file for testing:  ```ros2 launch cubemars_test cubemars_test.launch.py```
* This is a simple package containing only a single motor, with the position_controller control server (and only the steering motors have been tested so far)
* The URDF contains a hardware interface - ```<hardware>``` tags inside ```ros2_control``` - with position, velocity, effort and acceleration command interfaces.
* We have chosen to claim only the position command interface as of now, others can be chosen inside the ```cubemars_system.yaml``` config file.
* Steering commands are posted to ```/position_controller/commands```.
* When switching wheels, only change the hardware parameters according to what's defined in the Upper Computer software:
```                               
 <param name="can_id">105</param>
 <param name="pole_pairs">21</param>
 <param name="gear_ratio">9</param>
 <param name="kt">0.1</param>
````
* Code to rotate 180 degrees in 0.1 radian increments for testing steering (via command line):
```
for angle in $(seq 0 0.1 1.57); do   ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "{ data: [${angle}] }";   sleep 0.1; done
```
* Code to print steering position: ```ros2 topic echo /joint_states```
* The hardware interface is defined in the ```cubemars_interface``` package (similar to ```cubemars_hardware``` but with required changes to the code)

GIT commands:
* Check which branch we're in (and which files have changed): ```git status```
* Checkout a branch: ```git checkout branch_name```
* Create and checkout branch: ```git checkout -b new_branch_name```
* Add to tracked items: ```git add .```
* Commit to branch: ```git commit -m "description" .```
* Push to remote branch: ```git push origin branch_name```
* Pull from remote branch: ```git pull origin branch_name```
