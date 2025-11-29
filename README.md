# Research Track 1 - Assignment 1

Student: Bahri Riadh
ID:8335614

## Project Description
This ROS 2 package implements a control system for Turtlesim with two nodes:

1.  **UI Node (Python):** A remote controller that allows the user to spawn a second turtle and send velocity commands to a specific turtle.
2.  **Distance Monitor (C++):** A safety node that monitors the distance between `turtle1` and `turtle2`. It automatically stops the robots if:
    * They get too close (Collision avoidance).
    * They get too close to the boundaries (Wall avoidance).

## How to Run
To run the full simulation, open please  3 separate terminals.

#### 1. Start the Simulation
```bash
ros2 run turtlesim turtlesim_node
```

#### 2. Spawn Turtle & Start Safety System
This spawns turtle2 and starts the C++ distance monitor.
```bash
ros2 run assignment1_rt turtle_spawn.py
ros2 run assignment1_rt distance_monitor
```

#### 3. Run the UI Controller
Use this terminal to send commands.
```bash
ros2 run assignment1_rt ui_node.py
```

## Files Structure
* `scripts/ui_node.py`: Python script handling user input and velocity commands.
* `scripts/turtle_spawn.py`: Python script to call the /spawn service.
* `src/distance_monitor.cpp`: C++ source code implementing the distance calculation and safety logic.
* `CMakeLists.txt`: Configuration for building the mixed C++/Python package.
* `package.xml`: Defines dependencies (rclcpp, rclpy, turtlesim, geometry_msgs, std_msgs).