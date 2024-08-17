# Swarm-Drone-Optimal-Path-Planning
A project focused on developing and implementing algorithms for efficient exploration by swarm drones in emergency situations, such as firefighting. The project includes mathematical models and simulations to maximize ground coverage, maintain connectivity, manage battery power, and ensure efficient communication among drones.

## Project Overview

This project aims to develop a framework for optimal path planning in swarm drones to assist in emergency response scenarios, such as firefighting. The primary objectives are to:

- Maximize ground coverage.
- Ensure continuous connectivity for data sharing.
- Manage battery power efficiently.
- Implement and simulate the framework using well-known algorithms like Ant Colony Optimization (ACO) and RRT*.

## Key Features

- **Pheromone-Based Movement**: Drones deposit and follow pheromones to explore new areas while avoiding revisits.
- **Global and Local Path Planning**: Using a combination of k-means clustering and pheromone maps for efficient navigation.
- **Adaptive Speed Control**: Adjust drone speed based on current battery levels to prolong mission duration.
- **Efficient Communication**: Maintain strong network connectivity within the swarm through periodic updates and distance-weighted metrics.

## RRTstar.m
This MATLAB script implements 3D path planning for a swarm of drones using the Rapidly-exploring Random Tree Star (RRT*) algorithm. The primary objective is to navigate each drone through a set of predefined waypoints while ensuring that all drones converge at a common landing point. To run this code we use Matlab online IDE. 
The code begins by loading a 3D occupancy map (omap) representing a city block environment. The occupancy map is used to define the free space and obstacles that the drones must navigate through. The occupancy map is inflated to create a buffer zone around obstacles, ensuring that drones maintain a safe distance from these obstacles during navigation. An SE(3) state space is defined, which includes the position (x, y, z) and orientation (roll, pitch, yaw) of the drones. The bounds are set to confine the drones within the dimensions of the map. The state validator (sv) is created using the occupancy map and state space. It ensures that the planned paths are valid (i.e., they do not collide with obstacles). An RRT* path planner is set up with specific parameters to efficiently explore the state space and find an optimal path. The goal function determines when a drone is considered to have reached a waypoint. The script defines the initial positions (start poses) for each drone and a set of waypoints that the drones must visit. Each drone is assigned a subset of the waypoints. After visiting all assigned waypoints, all drones are directed to a common landing point.
The RRT* planner is used to generate a path for each drone to navigate through its waypoints and eventually reach the common landing point. The path for each drone is stored for visualization and analysis. The paths are interpolated to ensure smooth animation, and the movement of the drones is animated simultaneously. The positions of the drones are updated in real time during the animation.
The final paths taken by each drone are plotted in 3D space, providing a visual overview of the trajectory followed by each drone. The total path cost (sum of all segment costs for all drones) is calculated and displayed, giving an indication of the efficiency of the planned paths.



