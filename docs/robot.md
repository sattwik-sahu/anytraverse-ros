# Configuring Robots

Controllers for MOON Lab robots can be started using the `moonlab_robots` package.

## Usage

To start the _Hound_ robot, for example, first set the `MAVPROXY_CONNECTION_STR` with
```bash
export MAVPROXY_CONNECTION_STR=<YOUR_CONNECTION_STR>
```

then run

```bash
ros2 launch moonlab_robots hound_launch.yaml connection_str:=$MAVPROXY_CONNECTION_STR cmd_vel_topic:=/anytraverse/cmd_vel
```

> [!TIP]
> When working with MOON Lab robots in other frameworks, you may change the `cmd_vel_topic` argument to the relevant topics.

## Bring your Own Robot

Got your own robot you want to configure and navigate with AnyTraverse? We've got you covered! Just follow the steps below to get started. Let's assume your robot is named `mybot`, you may replace this with your robot's name and follow along.

### Configure AnyTraverse

First, you need to configure the initialization parameters for AnyTraverse specific to your robot.

1. Create the configuration file
    ```bash
    touch src/anytraverse_ros/config/params_mybot.yaml
    ```
2. Populate the file with the following. Tune the numbers to suit your robot.
    ```yaml
    /anytraverse_node:
        ros__parameters:
            roi_unc_thresh: 0.3
            scene_sim_thresh: 0.85
            roi_x_bounds: [0.35, 0.65]
            roi_y_bounds: [0.75, 1.00]
    ```

### Configure Nav2

Now that we have AnyTraverse set up for traversability segmentation, we need to configure the ROS2 Nav2 stack to navigate your robot to the goal safely using AnyTraverse. First, create your Nav2 config with

```bash
touch src/trav_map_navigation/config/params_mybot.yaml
```

We will not be covering Nav2 parameter tuning here. You may refer to the [official Nav2 documentation](https://docs.nav2.org/setup_guides/index.html) for a better understanding.

### Build Packages

Let's build the packages again to register the newly created configuration files.

```bash
colcon build --symlink-install
# OR
pixi run build
```

### We are Go For Launch! :rocket:

With everything about `mybot` configured, we are now ready to launch the navigation stack with AnyTraverse.

```bash
ros2 launch trav_map_navigation anyt_launch.py robot:=mybot init_prompt:="${INIT_PROMPT}"
```
