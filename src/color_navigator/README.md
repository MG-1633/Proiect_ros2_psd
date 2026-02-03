# Color Navigator Project

The Color Navigator project is a ROS2 package designed to simulate a robot in a Gazebo environment. The robot's task is to locate a colored box, identify the colors on its faces, and announce them. This project utilizes various ROS2 functionalities, including image processing and robot control.

## Project Structure

The project consists of the following main components:

- **ROS2 Package**: The package is named `color_navigator` and contains all necessary files for building and running the project.
- **Gazebo Models**: Two models are included:
  - A colored box with different colors on its faces.
  - A camera-equipped robot that can navigate and detect colors.
- **Gazebo World**: A simple world setup that includes the colored box and the robot.
- **Python Nodes**: Two nodes are implemented:
  - `color_detector_node`: Detects colors from the camera feed.
  - `robot_controller_node`: Controls the robot's movement based on detected colors.

## Installation

To set up the project, follow these steps:

1. **Install ROS2 Humble**: Ensure that you have ROS2 Humble installed on your system. Follow the official installation guide for your operating system.

2. **Clone the Repository**: Clone this repository to your ROS2 workspace:
   ```
   git clone <repository-url>
   ```

3. **Build the Package**: Navigate to your ROS2 workspace and build the package:
   ```
   cd <your_workspace>
   colcon build --packages-select color_navigator
   ```

4. **Source the Workspace**: After building, source the workspace:
   ```
   source install/setup.bash
   ```

## Usage

To launch the simulation and start the nodes, use the provided launch file:

```
ros2 launch color_navigator navigate_box.launch.py
```

This command will start the Gazebo simulation with the defined world and spawn the robot. The robot will begin searching for the colored box, detect the colors, and announce them.

## Dependencies

The project requires the following ROS2 packages:

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`

Ensure these dependencies are installed in your ROS2 environment.

## Contributing

Contributions to the Color Navigator project are welcome! If you have suggestions or improvements, please feel free to submit a pull request.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.