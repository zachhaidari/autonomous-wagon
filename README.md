# Autonomous Wagon - Hands-Free Load Transportation Robot

An autonomous mobile robot that follows a user hands-free using depth sensing, neural network-based target tracking, and closed-loop control. Designed to assist all users (able-bodied and disabled) in transporting items in personal, industrial, agricultural, and outdoor settings.

## Features

- **Autonomous Following**: Camera + depth sensor identifies and tracks user in real-time
- **Obstacle Avoidance**: Safety checks and obstacle detection prevent collisions
- **Closed-Loop Control**: Motor feedback and steering adjustment for smooth, safe following
- **Modular ROS 2 Architecture**: Separate packages for vision, control, sensors, and safety
- **Foldable Design**: Hardware contained to allow storage convenience
- **Simulation & Hardware Testing**: Supports both simulation and real hardware deployment
- **Data Logging**: Records runs for model improvement and control tuning

## System Architecture

```
Depth Camera → NN Inference → Target Tracking
                                    ↓
Depth Data → Distance Estimation ← Obstacle Detection
                ↓
        Motor Speed & Steering Control
                ↓
Encoder Feedback → Safety Manager → Closed-Loop Adjustment
```

## Hardware Components

| Component | Type | Qty | Cost |
|-----------|------|-----|------|
| Wagon (mechanical base) | Mechanical | 1 | $29.19 |
| Battery | Power | 1 | $80.00 |
| Scooter Motor & Driver (×2) | Drive | 2 | $0.00 |
| Raspberry Pi 4 Model B | Brain | 1 | $55.00 |
| Depth Camera | Sensor | 1 | $169.00 |
| Magnetic Encoder | Steer Sensor | 1 | $7.99 |
| DC to DC Buck Converter | Steer | 1 | $7.99 |
| PWM to DCR Converter | Drive | 1 | $13.48 |
| Stepper Motor Bipolar | Steer | 1 | $13.99 |
| Stepper Motor Driver | Steer | 1 | $10.89 |
| GPS Sensor | Sensor | 1 | $9.99 |
| IMU Sensor | Sensor | 1 | $20.00 |
| **TOTAL** | | | **$417.52** |

## Project Structure

```
autonomous-wagon/
├── src/
│   ├── wagon_core/           # Main coordinator, calibration tools
│   │   ├── package.xml
│   │   └── wagon_core/
│   ├── vision/               # Camera, depth sensor, NN inference
│   │   ├── package.xml
│   │   └── vision/
│   ├── control/              # Motor control, steering, closed-loop
│   │   ├── package.xml
│   │   └── control/
│   ├── safety/               # Obstacle detection, emergency stop
│   │   ├── package.xml
│   │   └── safety/
│   └── sensors/              # IMU, GPS, encoders
│       ├── package.xml
│       └── sensors/
├── config/                   # Configuration files (YAML)
├── launch/                   # ROS 2 launch files
├── README.md
└── .github/
    └── copilot-instructions.md
```

## Prerequisites

- **OS**: Ubuntu 22.04 LTS (ROS 2 Humble) or Ubuntu 24.04 (ROS 2 Iron)
- **Python**: 3.10+
- **ROS 2**: Humble or Iron distribution
- **Hardware**: Raspberry Pi 4, depth camera (e.g., Intel RealSense), sensors

## Installation

### 1. Install ROS 2
Follow [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for your platform.

### 2. Create Workspace
```bash
mkdir -p ~/Projects/autonomous-wagon
cd ~/Projects/autonomous-wagon
```

### 3. Install Dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 4. Build Project
```bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Source Setup
```bash
cd ~/Projects/autonomous-wagon
source install/setup.bash
```

### Run All Nodes
```bash
ros2 launch wagon_core wagon_system.launch.py
```

### Run Individual Nodes
```bash
# Vision/tracking
ros2 run vision target_tracker

# Motor control
ros2 run control motor_controller

# Safety manager
ros2 run safety safety_manager

# Sensor readers
ros2 run sensors imu_node
ros2 run sensors gps_node
```

### Monitor System
```bash
# View active topics
ros2 topic list

# Echo a topic
ros2 topic echo /target_pose

# View node graph
ros2 run rqt_graph rqt_graph
```

## Configuration

Edit YAML files in `config/` directory to adjust:
- Camera calibration parameters
- Motor speed limits
- Steering angles
- Safety thresholds (min following distance, obstacle detection range)
- Neural network model path

Example: `config/wagon_params.yaml`

## Data Logging

Runs are automatically logged using ROS 2 bag files:
```bash
# Record a run
ros2 bag record -a

# Playback
ros2 bag play rosbag2_data
```

## Testing

### Simulation
```bash
# Placeholder for simulation launch
ros2 launch wagon_core wagon_simulation.launch.py
```

### Hardware Testing
```bash
# Run on actual hardware
ros2 launch wagon_core wagon_hardware.launch.py
```

## Calibration

### Camera Calibration
```bash
ros2 run vision calibrate_camera
```

### Wheel Calibration
```bash
ros2 run control calibrate_wheels
```

## Development

### Adding New Nodes
1. Create subdirectory in appropriate `src/` package
2. Implement node with ROS 2 Python client library
3. Add entry point in `setup.py`
4. Update launch files

### Code Style
- Follow PEP 8 for Python
- Use type hints
- Document with docstrings

## Safety Features

- **Obstacle Detection**: Depth sensor scans for obstacles in path
- **Data Validation**: Staleness checks for sensor data
- **Emergency Stop**: Triggered on safety violations
- **Closed-Loop Verification**: Ensures commands are executed as intended
- **Timeout Protection**: Stops motors if no new commands received

## Future Enhancements

- [ ] Multi-target following (group support)
- [ ] Advanced path planning (navigate around obstacles)
- [ ] Improved neural network models
- [ ] Mobile app for remote monitoring
- [ ] Battery management and low-power warnings
- [ ] Weather-resistant enclosure
- [ ] Autonomous docking/charging

## Contributing

1. Create feature branch
2. Make changes with clear commit messages
3. Test thoroughly on hardware and simulation
4. Submit pull request with documentation

## License

[Specify license here]

## Contact & Support

For questions or issues, please open a GitHub issue or contact the project team.

---

**Status**: Early Development  
**Last Updated**: December 2025
