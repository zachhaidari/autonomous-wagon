# ROS 2 Autonomous Wagon Project

This is a ROS 2-based project for an autonomous following wagon system.

## Quick Reference

**Build the workspace:**
```bash
cd ~/Projects/autonomous-wagon
colcon build --symlink-install
```

**Source the workspace:**
```bash
source install/setup.bash
```

**Run the complete system:**
```bash
ros2 launch wagon_core wagon_system.launch.py
```

**View the node graph:**
```bash
ros2 run rqt_graph rqt_graph
```

**Record a data run:**
```bash
ros2 bag record -a
```

## Project Structure

- `src/` - ROS 2 packages
  - `wagon_core/` - Main coordinator
  - `vision/` - Camera and neural network tracking
  - `control/` - Motor and steering control
  - `safety/` - Safety manager and obstacle detection
  - `sensors/` - IMU, GPS, encoder drivers
- `config/` - YAML configuration files
- `launch/` - ROS 2 launch files

## Next Steps

1. Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
2. Implement node logic (see TODO comments in code)
3. Connect hardware interfaces
4. Test with simulation
5. Deploy to hardware

See README.md for full documentation.
