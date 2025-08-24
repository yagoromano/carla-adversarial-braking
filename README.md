# Carla YOLO Test Node

Put this directory in the src folder with the carla-ros-bridge package. Then run the build process.

```tree
carla-ros-bridge
├── src
    ├── carla_autobreak_test
    ├── ros-bridge
```

```bash
colcon build
```

This should build all the ros-bridge packages and the carla_autobreak_test package.

After all the packages are built, source the packages.

```bash
source install/setup.bash
```

Start the Carla server and the carla_autobreak_test package.
```bash
cd path/to/carla
./CarlaUE4.sh
```

```bash
ros2 launch carla_autobreak_test carla_autobreak_test.launch.py
```

This will start the carla-ros-bridge and the yolo test node. The yolo test node will subscribe to the camera topic and publish the results on the /carla/ego_vehicle/yolo/image topic.

To vizualize the sensors start rviz2.

```bash
ros2 run rviz2 rviz2
```

Then use the add button in the Displays panel to add the desired sensors.
