Set up SLCAN:
Follow ```https://github.com/TAUverIL/Exodus2025/blob/main/mobility_ctrl/commands.md``` - go to SLCAN instructions
Password: tauver
To stop running: Ctrl+C in the terminal

Go to ROS folder: cd Exodus2025/mobility_ctrl/
Relevant package: cubemars_test
In VSCode, go to urdf folder and open urdf.
Code for one wheel:
1. Change the can_id in the URDF
2. Do ```source install/setup.bash```
3. run ```ros2 launch cubemars_test cubemars_test.launch.py``` and verify that you see "entered the while!"
4. run:
```
for angle in $(seq 0 0.1 1.57); do   ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray "{ data: [${angle}] }";   sleep 0.1; done
```
