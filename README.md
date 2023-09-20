# tc200_hda
This repository contains the ROS package for the tc200 robot. The ROS package is the implementation of a warehouse management system using the holonomic tc200 robot.


## Project tree
 * [params](./params)
   * [map_nav_params](./params/map_nav_params)
   * [odom_nav_params](./params/odom_nav_params)
 * [CMakeLists.txt](./CMakeLists.txt)
 * [include](./include)
   * [tc200_hda](./include/tc200_hda)
 * [package.xml](./package.xml)
 * [launch](./launch)
   * [user.launch](./launch/user.launch)
 * [maps](./maps)
 * [src](./src)
   * [user_side_scripts](./src/user_side_scripts)
     * [shelf_list](./src/user_side_scripts/shelf_list)
     * [sort.py](./src/user_side_scripts/sort.py)
     * [cv_models](./src/user_side_scripts/cv_models)
     * [object_tracker.py](./src/user_side_scripts/object_tracker.py)
     * [task_scheduler.py](./src/user_side_scripts/task_scheduler.py)
     * [main.py](./src/user_side_scripts/main.py)
     * [autonomous_navigation.py](./src/user_side_scripts/autonomous_navigation.py)
   * [tc200_scripts](./src/tc200_scripts)
   * [parallel_movement.py](./src/tc200_scripts/parallel_movement.py)
 * [tree-md](./tree-md)

## About TC200 robot
TC200 is a versatile and entirely customizable platform specifically tailored for educational and training purposes. 
It has mecanum wheels, or omnidirectional wheels which allows the robot with holonomic motion capabilities, making it exceptionally well-suited for 
asks within confined spaces with limited manoeuvring room.

