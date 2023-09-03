# F1TENTH-SLAM-and-Path-Tracking--Autonomous-Navigation-with-Pure-Pursuit
This repository contains the implementations and resources developed , focusing on the core principles of SLAM, localization using Particle Filters, and the Pure Pursuit algorithm.


### Overview
#### Autonomous Navigation with SLAM and Pure Pursuit
This project delves into the intricacies of autonomous navigation by bridging the power of SLAM and the Pure Pursuit path tracking technique.

## 1. SLAM (Simultaneous Localization and Mapping)

- **Toolkit:** The `slam_toolbox` was the tool of choice to generate a holistic and precise map. This tool was ideal for its efficiency and real-time mapping capabilities.
- **Outcome:** The end product was a mapped representation of Levine's second floor, encapsulated within the files `levine_2nd.pgm` (image file representing the map) and `levine_2nd.yaml` (metadata for the map).

## 2. Localization using Particle Filters

- **Concept:** Localization was achieved by distributing a set of particles over the map. As the robot moves, these particles predict the robot's new position based on its movement and sensor readings.
- **Implementation:** With the map of Levine's second floor at our disposal, the particle filter was programmed to assimilate and adjust particles, ensuring that the robot's position and orientation were continuously updated and accurate.

## 3. Pure Pursuit Path Tracking Algorithm

- **Overview:** The Pure Pursuit algorithm is a path tracking technique where the robot chases a point (goal) ahead in its path rather than the final destination, ensuring smoother paths and turns.
- **Algorithmic Detail:** The critical component in this algorithm is the calculation of the steering angle derived from the curvature, represented as:

γ = 2 * ∣y∣ / L

where `y` is the lateral offset of the robot from the path and `L` is the look-ahead distance.
- **Testing:** Initial tests were carried out in a simulator environment using the robot's ground truth pose for validation. When transitioning to real-world tests, the particle filter's localization output was integrated to offer real-time updates to the algorithm.

## 4. Logging and Interpolating Waypoints

- **Joystick-driven Path Recording:** A ROS node was crafted to subscribe to the robot's pose, updated by the particle filter. This real-time pose data was archived as waypoints in a `.csv` format.
- **Interpolated Spline Creation:** Leveraging landmarks, such as the corners of the Levine loop, interpolated splines were generated. These splines, constructed using `scipy.interpolate.splprep` and `scipy.interpolate.splev`, guaranteed fluid and realistic paths for the robot.

## 5. Visualization in RViz

- **Purpose:** Visualization is essential for debugging, validation, and real-time tracking.
- **Techniques:** This project integrated `visualization_msgs` messages within the ROS framework to project the list of waypoints. In tandem with RViz, a dynamic visualization of the currently pursued waypoint was displayed, aiding in real-time adjustments and validations.
- 
## Simulation and hardware results:

### Simulation in Rviz

https://youtu.be/Bi7jUtaWYeA

### Implementation in actual f1 tenth car

https://youtube.com/shorts/ONBXS3RRPFw?si=BXFFLPhO8Ma_g1Zz



The codebase encompasses a rich blend of robotics algorithms and techniques. It serves as a comprehensive resource for roboticists and researchers looking to delve deep into the realm of autonomous navigation, offering both code and conceptual depth.
