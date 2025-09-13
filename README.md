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

Modifications:
- in the `/launch` folder, a new file `spawn_turtlebot3_robot2.launch.py` has been created, based on the orginal file `spawn_turtlebot3.launch.py` included in the same folder. The new file allows to spawn the correct model of the robot and the correct bridge file;
- in the `/models/turtlebot3_burger` folder, a new file `model_robot2.sdf` has been created, based on the file `model.sdf`. Both files were generated starting from the original file `model.sdf` contained in the folder  `/models/turtlebot3_burger` in [ROBOTIS-GIT/turtlebot3_simulations](https://github.com/ROBOTIS-GIT/turtlebot3_simulations), modified for the correct management of the namespace in both gazebo and ROS2;
- 


