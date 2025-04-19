# DOK4: Software Repository for the 23rd Robot Aircraft Competition

This repository stores the software code and packages developed for participation in the **23rd Robot Aircraft Competition 2025**.

## Included Packages

- **multirotor_control**
  Contains offboard control example code for multirotors, using {WGS84} coordinates to perform long-range autonomous flight.

- **fixedwing_control**
  Contains offboard control example code for fixed-wing aircraft, using {WGS84} coordinates to perform long-range autonomous flight.

- **vtol_control**
  Implements and validates transition and reverse transition for VTOL, and contains offboard control example code that uses {WGS84} coordinates for long-range autonomous flight.

- **precision landing**
  This package is dedicated to precision landing mission scenario presented in the competition.

- **final_mission**
  The final integrated software package for the overall competition system.

## Development Environment and Dependencies

- **OS**: Ubuntu 22.04 LTS
- **ROS2 Distribution**: Humble
- **Python**: 3.10
- **Flight Control**: PX4
- **Simulation**: Gazebo Harmonic (or Gazebo Classic)
- **Messaging Package**: px4_msgs

## Usage

Execute the following commands in your terminal to clone, build, and launch the project:

```bash
$ git clone https://github.com/kimhoyun-robotair/DOK4.git
$ cd DOK4
$ colcon build --symlink-install
$ ros2 launch final_mission final_mission.launch.py
