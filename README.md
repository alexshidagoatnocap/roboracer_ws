# My F1Tenth / Roboracer Workspace

This is where I keep all my ROS2 packages and nodes for the F1Tenth/Roboracer competition.

This is based on top of ROS2 Foxy with Ubuntu 20.04, a Docker image and Docker Compose file have been provided for running the workspace in a containerized environment.

Completed Nodes:
- Automatic Emergency Braking (safety_node.py)
- PID Wall Follower (wall_follow.py)
- Follow the Gap (reactive_node.py)

Working On:


To Be Completed:
- Pure Pursuit Controller
- Stanley Controller
- Pathfinding with RRT/RRT*
- MPC

## Prerequisites for Installation

In order to run the workspace yourself, you will need:
- Docker
- Docker Compose

You will first need to [setup the F1Tenth Gym ROS2 bridge and simulator in a Docker container. ](https://github.com/f1tenth/f1tenth_gym_ros?tab=readme-ov-file#without-an-nvidia-gpu) Follow the steps to set it up ***Without an Nvidia GPU***.

## Workspace Setup Instructions

1. Clone the repo and enter into the ```roboracer_ws``` directory.

```bash
git clone https://github.com/alexshidagoatnocap/roboracer_ws
```
```bash
cd roboracer_ws
```

2. Build the container image and start a workspace container using Docker Compose. If you followed the steps to set up the ROS2 gym bridge and simulator you should have a network setup between the noVNC, sim workspace, and roboracer workspace containers.

```bash
docker-compose up -d
```

3. Attach to the currently running workspace container. The ROS2 setup should automatically sourced.

```bash 
docker attach roboracer_ws
```

***OPTIONAL:*** *You can also start a new terminal session using ```docker exec -it```, but you will need to source the ROS2 setup again.*

```bash
docker exec -it roboracer_ws bash
```

```bash
source /opt/ros/foxy/setup.bash
```

4. Build the workspace. You should see some unused parameter warnings because the C++ nodes from the F1tenth/Roboracer Lab templates have not been used.

```bash
colcon build
```

5. Source the local setup for the workspace.

```bash
source install/local_setup.bash
```

6. Bring up the F1tenth Gym Simulator container and run whichever nodes you want. For example, if you wanted to run the gap follow node, you would run:

```bash
ros2 run gap_follow reactive_node.py
```





