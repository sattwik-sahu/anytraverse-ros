# AnyTraverse ROS

ROS Package for offroad navigation with AnyTraverse

---

## Installation

> Coming Soon...

## Usage

### With OAK-D Camera

1. Start oakd node with
    ```bash
    oakd-vio-zmq oakd
    ```
2. Start the AnyTraverse + OAK-D navigation launch file with
    ```bash
    pixi run ros2 launch trav_map_navigation anyt_oakd_nav_launch.yaml
    ```
3. Start the traversability map navigation launch file, specifying the right obstacle points topic name
    ```bash
    pixi run ros2 launch trav_map_navigation navigation_launch.py obstacle_topic:=/anytraverse/obstacle_points
    ```
