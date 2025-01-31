# Robot Navigation System

## Overview

The Robot Navigation System is designed to enable a robot to move through a 2D grid environment while avoiding obstacles and respecting boundaries. The system provides precise control over the robot's movement, including the calculation of steps and rotation required to reach a target point.

## Key Features

1. **A* Pathfinding Algorithm:**
   - The system uses the **A* (A-star) algorithm** to find the shortest path between the robot's current position and the target. This widely used algorithm evaluates the cost of moving from the start to the goal, factoring in the distance traveled (`g_score`) and a heuristic estimate of the remaining distance (`h_score`), combining them into an `f_score`. The robot follows the path with the lowest total cost.
   
2. **Hierarchical Pathfinding Abstraction:**
   - In addition to A*, the system employs a **Hierarchical Pathfinding** abstraction, which divides the map into smaller, hierarchical zones for more efficient pathfinding. Instead of calculating paths at the grid level, the system first calculates paths at a higher level (across zones or regions). Then, it refines the path at the grid level within these zones, which improves performance for large environments. This abstraction helps in dealing with larger maps and more complex scenarios by simplifying the pathfinding problem.

3. **Rotation Angle Calculation:**
   - The robot calculates the angle required to face the target point, adjusting for the current orientation to ensure efficient and accurate navigation.

4. **Grid-based Movement:**
   - The robot moves on a grid, where each cell has a fixed size. The system ensures that the robot stays within the grid boundaries and avoids invalid movements.

5. **Boundary Handling:**
   - The robot respects predefined boundaries in the environment, ensuring that it does not attempt to move outside of the valid grid area.

6. **Direction Control:**
   - The system carefully controls the robot's orientation, adjusting its direction based on the target location and current facing direction.

7. **Step Calculation:**
   - The robot calculates the number of steps to reach the target using Euclidean distance. This allows the robot to determine how far it needs to travel.
