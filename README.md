# DiffDriveAutoMappingRobot

This is a ROS 2 Jazzy robot package for differential-drive autonomous mapping using SLAM, Nav2, and a custom behavior tree.

---

## How to Build and Run

###  Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Jazzy Jalisco installed:
  ```bash
  sudo apt update
  sudo apt install ros-jazzy-desktop
  ```

### Install ROS 2 Dependencies

From your ROS 2 workspace root (`~/ros2_ws`):

```bash
sudo apt update
rosdep update
rosdep install --from-paths src -i -y
```

---

### Build the Package

1. Source ROS 2:
   ```bash
   source /opt/ros/jazzy/setup.bash
   ```

2. Build only the `robot` package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select robot
   ```

3. Source your workspace:
   ```bash
   source install/setup.bash
   ```

---

### Launch the Robot

#### Mapping with SLAM + Nav2

Launch the full simulation with behavior tree exploration and mapping:
```bash
ros2 launch robot new_car_launch.py
```

### Runtime Tips

- RViz config: `config/robot_config.rviz`
- AprilTag parameters: `config/apriltags.yaml`
- EKF parameters: `config/ekf.yaml`
- Nav2 parameters: `config/nav2_params.yaml`
- SLAM parameters: `config/mapper_params_localization.yaml`

---

### Optional Cleanup

To remove old build artifacts:
```bash
cd ~/ros2_ws
rm -rf build/ install/ log/
```

---

Last updated: April 21, 2025
