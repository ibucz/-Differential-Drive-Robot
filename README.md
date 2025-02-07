# Differential Drive Robot

This project implements a **Differential Drive Robot** using **ROS1**, focusing on motion control, URDF modeling, and sensor-based mapping. The system integrates multiple sensors for environment perception and autonomous navigation.

---

## Features

- **Motion Control:**
  - Implemented kinematics and dynamics for differential drive movement.
  - Controlled robot velocity and trajectory using ROS.

- **URDF Modeling:**
  - Designed and simulated a **URDF (Unified Robot Description Format)** model for visualization in **RViz**.
  - Defined robot parameters including wheelbase, joints, and collision properties.

- **Sensor-Based Mapping:**
  - Utilized **IR and Ultrasonic Sensors** for obstacle detection and environment mapping.
  - Implemented **SLAM (Simultaneous Localization and Mapping)** for real-time tracking.

---

## Tools & Technologies

- **Robot Operating System (ROS1)**
- **Gazebo & RViz** for simulation and visualization
- **URDF** for robot modeling
- **IR & Ultrasonic Sensors** for mapping
- **Encoders** for position tracking

---

## Project Structure

```
Differential-Drive-Robot/
│-- src/        # ROS1 package source files
│-- urdf/       # Robot description files (URDF & Xacro)
│-- scripts/    # Motion control and navigation scripts
│-- launch/     # Launch files for simulation and testing
│-- docs/       # Documentation and implementation details
│-- README.md   # Project description and setup guide
```

---

## How to Use

1. Clone this repository:
   ```sh
   git clone https://github.com/ibucz/Differential-Drive-Robot.git
   ```
2. Set up **ROS1** and source the workspace:
   ```sh
   source devel/setup.bash
   ```
3. Launch the robot in simulation:
   ```sh
   roslaunch differential_drive_robot simulation.launch
   ```
4. Control the robot using keyboard or autonomous navigation.

---

## License

This project is licensed under the **MIT License**.

---

## Contributors
- [AHMED ABDULAZIZ](https://github.com/ibucz)

Feel free to contribute to this project by submitting issues or pull requests! 🚀

