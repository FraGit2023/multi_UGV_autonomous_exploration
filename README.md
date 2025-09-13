# 🤖 Multi-TurtleBot3 Autonomous Exploration with ROS 2 and Gazebo

## 📂 Project Structure

```bash
.
├── docker_ws/       # Docker workspace for building the development container
├── ros_ws/          # Main ROS 2 workspace containing all custom and third-party packages
├── chown_me.sh      # Script to fix ownership of files created as root inside the container
├── run.sh           # Script to run the Docker container with correct volumes and permissions
└── exec.sh          # Script to open a shell into a running container
```
### **`ros_ws/src`** contains:

#### **`turtlebot3_gazebo`** 
- Forked and adapted from [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
- Includes robot models and launch files to spawn TurtleBot3 in a Gazebo simulation and establish communication with ROS.


