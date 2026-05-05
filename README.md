 # CSE180-MRTP-Final-Project-Human-Tracker-Navigation

## 1. Problem Statement
The objective of this project was to develop a robust **ROS 2 controller** for a mobile robot tasked with patrolling a warehouse environment. The system autonomously navigates between points of interest and tracks human workers in real time.

### Core Requirements:
* **Autonomous Navigation:** Utilizes `nav2` for patrolling predefined waypoints to monitor two human workers (Human 1 and Human 2).
* **Human Detection:** Determines whether each human is present at their expected station or has moved.
* **Position Estimation:** If a human has moved, the system estimates their new position using Lidar data, effectively distinguishing humans from walls and background clutter.

## 2. Initial Approach & Basic Architecture
The solution is implemented in the `human_tracker` package using **C++** and **ROS 2 Humble**.

* **Navigation:** A dedicated mission thread sends goals to the `navigate_to_pose` action server. This ensures the robot patrols continuously without blocking the sensor processing loop.
* **Perception:** The node subscribes to `/scan` (Lidar) and `/map` (Static OccupancyGrid). By comparing real-time hits against the static map, hits falling in "free" space are flagged as dynamic obstacles (potential humans).

## 3. Challenges Encountered & Technical Solutions

### Challenge A: "Ghost Detections" & False Positives
**Issue:** The robot initially reported humans as "moved" immediately upon startup or when at long distances. It also frequently confuses walls for humans.
* **Solution 1: Proximity Gating:** Implemented a strict distance check. The robot only evaluates a human's status if it is within 4.5 meters of the target.
* **Solution 2: Smart Clustering (Euclidean + Width Filter):**
    * Raw Lidar points are grouped into clusters using Euclidean distance.
    * **Geometric Filtering:** The system calculates the width of each cluster. If it is wider than 0.9m, it is classified as Wall/Clutter and ignored.

### Challenge B: Human 2 Unreachable (Path Planning)
**Issue:** The direct path from Human 1 to Human 2 was blocked by walls, causing the `nav2` planner to struggle and the robot to get stuck.
* **Solution: Explicit Waypoint Routing:** Implemented a custom route that guides the robot through open aisles: `Start` -> `Center(0,0)` -> `Top(0,10)` -> `Waypoints around the top wall` -> `Human 2`.

### Challenge C: Identity Confusion
**Issue:** When Human 1 was missing, the robot would scan the map and incorrectly identify a distant Human 2 as the "moved" Human 1.
* **Solution: Maximum Search Radius:** Added a 6.0m limit for assigning new positions, preventing the robot from misidentifying clusters outside the general vicinity of the target.

### Challenge D: Noisy Position Estimation
**Issue:** Estimated coordinates for a moved human jumped around due to single-frame Lidar noise.
* **Solution: Low-Pass Filter (Continuous Averaging):** Instead of overwriting positions instantly, the system uses an exponential moving average:
    `NewPos = (0.8 * OldPos) + (0.2 * Measurement)`
    This results in a stable, smooth estimation that converges on the true location.

## 4. Final System Performance
The final system is highly robust:
* **Navigation:** Successfully navigates the full patrol loop while avoiding static obstacles.
* **Detection Accuracy:** Near-perfect classification of humans vs. walls using the filtering pipeline.
* **Estimation:** The Low-Pass Filter yields a consistent and reliable location for tracking purposes.

## 5. Conclusion
This project demonstrates the integration of autonomous navigation with raw sensor processing. By utilizing a multi-stage filtering pipeline (**Map Filter -> Euclidean Cluster -> Width Check -> Distance Check -> Low-Pass Filter**), the system is resilient to the noise and complexity of a warehouse environment.
