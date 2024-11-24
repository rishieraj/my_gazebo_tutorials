# ROS2 Gazebo Tutorials

This branch contains a C++ package that can run a ROS2 node to control a Turtlebot Waffle in a custom world.

**Author:** Rishie Raj (120425554)

### Dependencies
It is assumed that the user has ROS2 Humble installed in their local system and sourced in the environment. The code dependes on the following ROS2 packages:  
 - `ament_cmake`: Required for ROS2 build system integration.
 - `rclcpp`: Used for creating nodes and communication between them.
 - `std_msgs`: Provides standard message types used in ROS2.
 - `geometry_msgs`: Provides geometry type messages for ROS2.
 - `sensor_msgs`: Provides message type for sensors like LIDAR.

### Building the Code

```bash
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir -p ~/ros2_ws/src
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws/src
# Clone the repository
git clone git@github.com:rishieraj/my_gazebo_tutorials.git
# Go back to the ws directory
cd ~/ros2_ws
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select walker
# After successfull build source the package
source install/setup.bash
# Launch the walker node in terminal
ros2 launch walker walker_world.launch.py
```

### Checking ROS2 Bag Functionality

The ROS2 Bag functionality is checked by capturing the messages while running the nodes. It can be recorded by running the following codes

```bash
# Source the overlay in a new terminal window
source /opt/ros/humble/setup.bash
# Go back to the ws directory
cd ~/ros2_ws
# After successfull build source the package
source install/setup.bash
# Launch the walker node in terminal
ros2 launch walker walker_world.launch.py ros2_bag_start:=true
```

Then in another terminal, run the following
```bash
# Source the package
source install/setup.bash
# Inspect the ros2 bag
ros2 bag info walkerbag
# Play back the contents of the ros2 bag
ros2 bag play walkerbag
```
