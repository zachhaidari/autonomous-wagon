# Autonomous Wagon - ROS 2 Project

## Project Overview
Autonomous following wagon system that uses computer vision (depth camera + neural network) to track and follow a user while avoiding obstacles. The system uses ROS 2 for orchestration across all modules including motor control, sensors (IMU, GPS), safety checks, and closed-loop steering.

## Architecture
- **Vision Module**: Camera/depth sensor integration with neural network inference
- **Control Module**: Motor controller, steering actuator, closed-loop feedback
- **Sensor Module**: IMU, GPS, magnetic encoders, hall sensors
- **Safety Module**: Obstacle detection, data validation, emergency stop
- **Core**: Main coordinator and calibration tools

## Key Technologies
- ROS 2 Jazzy
- Python 3.12
- PyTorch/TensorFlow for NN inference
- OpenCV for vision processing
- Hardware: Raspberry Pi 4, depth camera, stepper motors, IMU, GPS

## Setup Progress
- [x] Project structure created
- [ ] ROS 2 packages configured
- [ ] Dependencies installed
- [ ] Core nodes implemented
- [ ] Integration testing
