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
