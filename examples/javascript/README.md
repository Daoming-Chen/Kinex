# Kinex UR5 Visualizer

This example demonstrates how to use Kinex with Three.js to visualize a UR5 robot and perform Inverse Kinematics (IK) interactively.

## Prerequisites

-   A web server to serve the files (due to CORS and module loading).
-   An internet connection to load `@kinex/wasm` from unpkg.
-   The UR5 model must be available at `../models/ur5/`.

## How to Run

1.  Open a terminal in the root of the Kinex repository (`~/Dev/Kinex`).
2.  Start a simple HTTP server:
    ```bash
    python3 -m http.server 8000
    ```
3.  Open your browser and navigate to:
    [http://localhost:8000/examples/javascript/index.html](http://localhost:8000/examples/javascript/index.html)

## Features

-   **Visualization**: Loads the UR5 URDF and meshes using Three.js.
-   **Interaction**: A red sphere represents the target position and orientation. Drag it to move the robot.
-   **Inverse Kinematics**: The robot's joints are updated in real-time using Kinex's `SQPIKSolver` to track the target.
-   **Forward Kinematics**: The robot's visual links are updated using Kinex's `ForwardKinematics`.
