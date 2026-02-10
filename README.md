# ROS 2 Humble Development Environment

This repository provides a containerized development environment for ROS 2 Humble, specifically configured for TurtleBot 3 simulation and navigation. It uses **Docker Compose** to manage the container lifecycle and a helper script to simplify terminal access.

## 📂 File Structure

* **`Dockerfile_current`**: The base image using `osrf/ros:humble-desktop`. It installs TurtleBot 3 packages, Gazebo, Navigation2, and essential tools like `transforms3d`.
* **`docker-compose.yml`**: Configures the container to use `host` networking and maps your local workspace directories to the container.
* **`docker_terminal.sh`**: A helper script that handles X11 permissions for GUI apps and enters the container environment.
* **`ros2_ws/`**: Your local workspace directory where your code resides.

---

## 🚀 Getting Started

### 1. Prerequisites
Ensure you have Docker and Docker Compose installed on your host machine.

### 2. Launching the Environment
You do not need to build the image manually using complex commands. Use the provided shell script to automate the process:

```bash
chmod +x docker_terminal.sh
./docker_terminal.sh
```

**This script performs the following:**

*   Ensures a .bash\_history file exists so your command history is saved.
    
*   Runs xhost +local:docker to allow GUI tools (Gazebo/Rviz) to open on your host screen.
    
*   Starts the container in "detached" mode using docker compose up -d.
    
*   Logs you into the container terminal as the user **student**.
    

### 3\. Building Your Workspace

Once inside the container terminal, navigate to your workspace and build your packages using colcon:

```bash
cd ~/ros2_ws  
colcon build --symlink-install  
source install/setup.bash   
```


🛠 Environment Details
----------------------

The environment is pre-configured with the following settings in your .bashrc:

*   **User**: student (Non-root user with sudo privileges).
    
*   **ROS\_DOMAIN\_ID**: 30 (Prevents interference with other students on the network).
    
*   **TURTLEBOT3\_MODEL**: burger (Default robot model for simulations).
    
*   **ROS\_LOCALHOST\_ONLY**: 1 (Limits ROS 2 traffic to the local machine).
    

⚠️ Important Notes
------------------

*   **Persistence**: Only files saved inside ~/ros2\_ws are stored on your physical computer. Other changes are lost when the container is removed.
    
*   **GUI Tools**: To run Gazebo or Rviz2, simply type the command in the terminal. The display is redirected to your host machine via the DISPLAY environment variable.
    
*   **Permissions**: If you need to install extra software, use sudo apt update. The student user has passwordless sudo access.