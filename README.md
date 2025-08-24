# Carla Autobreaking Node

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

This should build all the ros-bridge packages and the carla\_autobreak\_test package.

After all the packages are built, source the packages.

```bash
source install/setup.bash
```

Start the Carla server and the carla\_autobreak\_test package.

```bash
cd path/to/carla
./CarlaUE4.sh
```

```bash
ros2 launch carla_autobreak_test carla_autobreak_test.launch.py
```

This will start the carla-ros-bridge and the yolo test node. The yolo test node will subscribe to the camera topic and publish the results on the /carla/ego\_vehicle/yolo/image topic.

To visualize the sensors start rviz2.

```bash
ros2 run rviz2 rviz2
```

Then use the add button in the Displays panel to add the desired sensors.

---


Additional Nodes

In addition to the YOLO-based perception and vehicle control nodes described in the paper, this repository also includes new experimental nodes that were introduced later:

lidar_fusion.py – Fuses LiDAR and camera data for object distance estimation and visualization.

lidar.py – Performs object detection and clustering using LiDAR point clouds.

lidar_control.py – Controls the vehicle using LiDAR-based distance estimation with HMAC-based integrity checks.

depth.py – Processes depth camera images to estimate object distance.

lidar_topic_attack.py – Simulates spoofed LiDAR distance values to test robustness against adversarial sensor input.

Note: The main ISCAS 2025 paper only discusses the YOLO perception nodes and the vehicle control node. The additional nodes above were developed afterward to extend functionality and test fusion- and security-related scenarios.

## Citation

If you use this repository in your research, please cite the following paper:

```
@inproceedings{martinez2025mitigation,
  title={Mitigation of Camouflaged Adversarial Attacks in Autonomous Vehicles--A Case Study Using CARLA Simulator},
  author={Martinez, Yago Romano and Brady, Carter and Solanki, Abhijeet and Al Amiri, Wesam and Hasan, Syed Rafay and Guo, Terry N},
  booktitle={2025 IEEE International Symposium on Circuits and Systems (ISCAS)},
  pages={1--5},
  year={2025},
  organization={IEEE}
}
```
