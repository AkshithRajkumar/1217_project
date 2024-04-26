Planning Function Documentation
Overview
The planning function is responsible for generating waypoints and planning the trajectory for the quadrotor based on a sequence of gates while considering the uncertainty of obstacles. This function utilizes various algorithms, such as A* search, to find an optimal path from the starting point to the destination gate.

Input
- use_firmware: A boolean flag indicating whether to use the on-board firmware for control or a software-only alternative.
- initial_info: A dictionary containing a priori scenario information, including gate positions, gate types, obstacle positions, and other relevant parameters.
- initial_obs: The initial observation of the quadrotor's state, including position and velocity.
Output
- t_scaled: An array representing the scaled time steps for the trajectory.
- ref_x, ref_y, ref_z: Arrays representing the reference trajectory points along the x, y, and z axes, respectively.
Workflow
- Initialization: Initialize necessary parameters and variables based on the provided scenario information and initial observation.
- Generation of Waypoints:
    - Determine waypoints based on gate positions and obstacle avoidance constraints.
    - Calculate intermediate waypoints to ensure smooth trajectory transitions between gates.
    - Generate peripheral coordinates around obstacles to account for uncertainty.
- Trajectory Planning:
    - Utilize A* search algorithm to find the optimal path from the starting point to each gate.
    - Perform obstacle avoidance by considering obstacle positions and gate entry/exit points.
    - Generate a sequence of waypoints for the quadrotor to follow, ensuring safety and efficiency.
- Visualization:
    - Plot the generated trajectory in each dimension (x, y, z) and in 3D space.
    - Draw the trajectory on PyBullet's GUI for visualization and verification.
Notes
- The function incorporates gate positions, gate types, obstacle positions, and other scenario-specific parameters to plan a safe and efficient trajectory.
- Various algorithms, including A* search, are utilized for pathfinding and obstacle avoidance.
- The resulting trajectory is visualized for verification and further analysis.
