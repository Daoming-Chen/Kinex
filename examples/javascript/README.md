# Kinex UR5 Visualizer

This example demonstrates how to use Kinex with Three.js to visualize a UR5 robot and perform Inverse Kinematics (IK) interactively.

## Live Demo

Visit the live demo at: [https://Daoming-Chen.github.io/Kinex/](https://Daoming-Chen.github.io/Kinex/)

## Prerequisites

-   A web server to serve the files (due to CORS and module loading).
-   An internet connection to load `@daoming.chen/kinex` from npm.
-   The UR5 model files (included in this example).

## How to Run Locally

1.  Open a terminal in the `examples/javascript` directory.
2.  Start a simple HTTP server:
    ```bash
    python3 -m http.server 8000
    ```
3.  Open your browser and navigate to:
    [http://localhost:8000](http://localhost:8000)

## Features

-   **Visualization**: Loads the UR5 URDF and meshes using Three.js.
-   **Interaction**: A red sphere represents the target position and orientation. Drag it to move the robot.
-   **Inverse Kinematics**: The robot's joints are updated in real-time using Kinex's `SQPIKSolver` to track the target.
-   **Forward Kinematics**: The robot's visual links are updated using Kinex's `ForwardKinematics`.

## Deployment

This example is automatically deployed to GitHub Pages via GitHub Actions when changes are pushed to the `feat/github-pages-demo` branch.
