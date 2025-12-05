# Autonomous Wagon - Setup Guide

Complete step-by-step instructions for setting up the autonomous wagon ROS 2 project on a fresh Ubuntu 24.04 system.

## Prerequisites

- **OS**: Ubuntu 24.04 LTS
- **Internet Connection**: Required for package downloads
- **sudo Access**: Required for system package installation

## Step 1: Install ROS 2 Jazzy

### 1.1 Set up the ROS 2 repository

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release ubuntu-keyring
```

### 1.2 Add the ROS 2 GPG key

```bash
curl -sSL https://repo.ros2.org/ros.key | sudo apt-key add -
```

### 1.3 Add the ROS 2 repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### 1.4 Install ROS 2 Jazzy

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop
```

This installs:
- ROS 2 core libraries
- rclpy (Python client library)
- Common message types
- Developer tools

### 1.5 Verify ROS 2 Installation

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

Expected output: `release 0.X.X` (version number)

### 1.6 Make ROS 2 available in all new terminals (optional but recommended)

Add this line to your `~/.bashrc`:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 2: Install colcon Build Tool

### 2.1 Install colcon and required packages

```bash
sudo apt install -y python3-colcon-common-extensions python3-pip
```

### 2.2 Verify colcon installation

```bash
colcon --version
```

Expected output: `colcon X.X.X`

## Step 3: Clone and Build the Autonomous Wagon Project

### 3.1 Clone the repository

```bash
git clone https://github.com/zachhaidari/autonomous-wagon.git
cd autonomous-wagon
```

### 3.2 Install ROS 2 dependencies

```bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y
```

This installs all ROS 2 dependencies specified in the package.xml files.

### 3.3 Build all 5 packages with symlink-install

```bash
colcon build --symlink-install
```

This command:
- **Builds** all packages (control, safety, sensors, vision, wagon_core)
- **Uses symlink-install**: Changes to Python source files take effect immediately without rebuilding

Expected output:
```
Starting >>> control
Starting >>> safety
Starting >>> sensors
Starting >>> vision
Starting >>> wagon_core
Finished <<< control [X.XXs]
Finished <<< safety [X.XXs]
Finished <<< sensors [X.XXs]
Finished <<< vision [X.XXs]
Finished <<< wagon_core [X.XXs]

Summary: 5 packages finished [X.XXs]
```

### 3.4 Source the workspace setup

```bash
source install/setup.bash
```

Make this permanent by adding to `~/.bashrc`:

```bash
echo "source ~/autonomous-wagon/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Step 4: Verify the Installation

### 4.1 List available ROS 2 packages

```bash
ros2 pkg list | grep -E "(wagon|control|vision|safety|sensors)"
```

Expected output (5 packages):
```
control
safety
sensors
vision
wagon_core
```

### 4.2 Test the build

```bash
ros2 launch wagon_core wagon_system.launch.py
```

The system should start all 10 nodes successfully. You'll see:
```
[INFO] [target_tracker-1]: process started with pid [XXXXX]
[INFO] [depth_processor-2]: process started with pid [XXXXX]
[INFO] [imu_node-3]: process started with pid [XXXXX]
...
[INFO] [wagon_coordinator-10]: process started with pid [XXXXX]
```

Press `Ctrl+C` to stop the system.

### 4.3 Monitor topics (optional)

Open a new terminal and run:

```bash
source ~/autonomous-wagon/install/setup.bash
ros2 topic list
```

## Troubleshooting

### Issue: `ros2 command not found`
**Solution**: Source ROS 2 setup file
```bash
source /opt/ros/jazzy/setup.bash
```

### Issue: `colcon command not found`
**Solution**: Install colcon
```bash
sudo apt install -y python3-colcon-common-extensions
```

### Issue: Build fails with "package not found"
**Solution**: Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Issue: Launch file errors
**Solution**: Ensure workspace is sourced after rebuild
```bash
cd ~/autonomous-wagon
source install/setup.bash
```

## Project Structure

```
autonomous-wagon/
├── src/
│   ├── control/              # Motor and steering control
│   ├── safety/               # Safety manager and obstacle detection
│   ├── sensors/              # IMU, GPS, encoders
│   ├── vision/               # Camera and target tracking
│   └── wagon_core/           # Main coordinator
├── config/                   # Configuration files (YAML)
├── install/                  # Installed packages (auto-generated)
├── build/                    # Build artifacts (auto-generated)
└── README.md                 # Project documentation
```

## Next Steps

1. **Explore the code**: Check out the Python files in `src/*/` for node implementations
2. **Run individual nodes**: `ros2 run <package> <node_name>`
3. **Monitor topics**: `ros2 topic echo /<topic_name>`
4. **View node graph**: `ros2 run rqt_graph rqt_graph`
5. **Implement hardware interfaces**: Complete the TODO comments in node files

## Additional Resources

- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [colcon Documentation](https://colcon.readthedocs.io/)
- [ROS 2 Python Tutorials](https://docs.ros.org/en/jazzy/Tutorials/Beginner-level/Writing-a-Simple-Py-Publisher-and-Subscriber.html)

## Support

For issues or questions:
1. Check existing GitHub issues
2. Review ROS 2 documentation
3. Open a new GitHub issue with details

---

**Last Updated**: December 4, 2025  
**ROS 2 Version**: Jazzy  
**Ubuntu Version**: 24.04 LTS  
**Python Version**: 3.12
