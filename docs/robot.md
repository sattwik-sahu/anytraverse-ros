# Starting Robots

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
