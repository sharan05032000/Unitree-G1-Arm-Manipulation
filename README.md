# Unitree G1 Right Arm MoveIt Configuration

This repository provides a **MoveIt 2 configuration and ROS 2 workspace** for controlling the **right arm of the Unitree G1 robot**. It enables simulation, motion planning, and control of the arm using ROS 2 Humble and MoveIt 2.

---

![Moveit Rviz](images/moveit.gif)

## Features

- Fully configured **MoveIt 2 setup** for the Unitree G1 right arm.
- Trajectory planning using interactive marker.
- Trajectory planning using  **3D pose input**.
- **Trajectory visualization** via `/display_planned_path`.


---

## Prerequisites

Before proceeding, ensure you have:

- **ROS 2 Humble** ([Installation Guide](https://docs.ros.org/en/humble/Installation.html))
- **MoveIt 2** ([MoveIt 2 Installation Guide](https://moveit.ros.org/install/))


---

## Workspace Setup

1. **Create the workspace:**
   ```bash
   mkdir -p ~/g1_ws/src
   cd ~/g1_ws/src
   ```

2. **Clone this repository:**
   ```bash
   git clone https://github.com/sharan05032000/Unitree-G1-Arm-Manipulation.git
   ```

3. **Build the workspace:**
   ```bash
   cd ~/g1_ws
   colcon build
   source install/setup.bash
   ```

---

## Launching the Right Arm Demo

To visualize and control the right arm using MoveIt 2 in RViz:

```bash
ros2 launch g1_arm demo.launch.py
```

- Use the interactive markers to plan and execute trajectories.

---

## Providing Custom 3D Poses

You can control the end-effector by running the included Python script:

```bash
python3 xyz_coordinate.py
```

When executed, the script will **prompt you to enter the target 3D coordinates (x, y, z)**  eg(0.2, 0, 0.2) for the robot’s end-effector.
After providing the values, MoveIt will plan and execute a trajectory to move the arm to the specified position.

>  You can modify the script to include orientation inputs or predefined poses if needed.

### Coordinate Reference



![Coordinate Frame](images/base_coord.png)

---

## Visualizing Planned Trajectories

To view the planned motion trajectory published by MoveIt:

```bash
ros2 topic echo /display_planned_path
```

This topic outputs the **planned joint trajectories**.


## Troubleshooting

If you encounter controller loader errors such as:

```
Loader for controller 'joint_state_broadcaster' not found.
Loader for controller 'gripper_control' not found.
Loader for controller 'right_control' not found.
```

Install the required ROS 2 control packages:

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
```

Then rebuild and source your workspace:

```bash
cd ~/g1_ws
colcon build
source install/setup.bash
```

---

## Extending the Setup

- The current configuration supports only the **right arm**.
- You can generate similar MoveIt configurations for other arms using **MoveIt Setup Assistant**.
- Update your URDF/Xacro to include additional joints, sensors, or grippers as needed.

---

## Repository Structure

```
g1_ws/
└── src
    ├── g1_arm
    │   ├── CMakeLists.txt
    │   ├── config
    │   ├── launch
    │   └── package.xml
    ├── g1_description 
    │   ├── CMakeLists.txt
    │   ├── meshes
    │   ├── package.xml
    │   ├── __pycache__
    │   └── urdf
    ├── README.md
    └── xyz_coordinate.py 

```

---

## References

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/index.html)
- [MoveIt 2 Documentation](https://moveit.ros.org/)

