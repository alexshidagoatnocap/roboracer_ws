# My F1Tenth / Roboracer Workspace

This is where I keep all my ROS2 packages and nodes for the F1Tenth/Roboracer competition.

This is based on top of ROS2 Foxy with Ubuntu 20.04, a Docker image along with instructions will be provided in the future to run the container in the F1Tenth simulator. 

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

## Instructions

More instructions on how to run this in a Docker container alongside with the F1tenth simulator will be provided in the future. 

The only way to currently use this workspace is to clone repo into Ubuntu 20.04 with ROS2 Foxy on it. Source the ROS2 Foxy setup using ```source /opt/ros/foxy/setup.bash``` and then cd into the workspace and run ```colcon build``` Once the workspace has built, source the workspace setup using ```source install/setup.bash``` and run whichever nodes you want alongside the F1tenth simulator.



