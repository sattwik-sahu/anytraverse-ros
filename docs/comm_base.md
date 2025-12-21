# Communication with Base Station

This tutorial guides you through starting navigation on your robot and visualizing topics through Foxglove on the base station machine.

> [!IMPORTANT]
> - Since we are using Pixi, it is necessary to set the `ROS_DOMAIN_ID` on both machines to ensure communication.
> - Please make sure both the robot and base station machines are connected to the same network before going ahead.
> - Although the communication will work on your mobile hotspot, we recommend using a separate router with during field deployment to avoid network transport bottlenecks during operation.

## On Robot

1. Set the `ROS_DOMAIN_ID`
    ```bash
    export ROS_DOMAIN_ID=7
    ```
2. Follow the steps [here](../README.md#usage) to launch AnyTraverse navigation on the robot.
3. Note down the `tcp/<ROBOT_IP>:<ROBOT_PORT>` printed by the `pixi run zenohd` command

## On Base Station

### Setup

- Make sure you have either:
    - Installed the repo following the steps [here](../README.md#installation), or
    - Sourced ROS2 and installed the Zenoh RMW with
        ```bash
        sudo apt install ros-$ROS_DISTRO-rmw-zenoh-cpp
        ```
- If you have not installed the repo, install the [ROS Foxglove bridge](https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge) with
    ```bash
    sudo apt install ros-$ROS_DISTRO-foxglove-bridge
    ```

### Visualize Robot Topics

1. Configure the Zenoh router on the base station to connect to the robot, using the [robot zenoh router address](#on-robot)
    ```bash
    export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["tcp/<ROBOT_IP>:<ROBOT_PORT>"]'
    ```
2. Set the `ROS_DOMAIN_ID` (Do this for every terminal)
    ```bash
    export ROS_DOMAIN_ID=7
    ```
3. Start the [Zenoh router](https://github.com/ros2/rmw_zenoh/?tab=readme-ov-file#examples) for the base station
    ```bash
    pixi run ros2 run rmw_zenoh_cpp rmw_zenohd   # with pixi
    ros2 run rmw_zenoh_cpp rmw_zenohd   # System ROS2
    ```
4. Check if the nodes are visible (on a different terminal)
    ```bash
    pixi run ros2 node list   # with pixi
    ros2 node list | grep anytraverse   # System ROS2
    ```
    You should see the `/anytraverse/...` nodes
5. Launch the ROS Foxglove bridge
    ```bash
    ros2 launch foxglove_bridge foxglove_bridge_launch.xml
    ```
6. Open Foxglove on your base station with the browser or the local app to view the topics (default: `localhost:8765`)
