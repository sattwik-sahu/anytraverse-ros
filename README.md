# AnyTraverse ROS

ROS Package for offroad navigation with [AnyTraverse](https://github.com/sattwik-sahu/AnyTraverse)

---

## Installation

> [!NOTE]
> This package is built currently for _x86 Linux_ and _arm64 OSX_ machines, windows supported may be added later

### Prerequisites

- Install the [Pixi](https://pixi.sh/) dependency manager. You may have to reopen your terminal for the changes to take effect.
    ```bash
    curl -fsSL https://pixi.sh/install.sh | bash
    ```
- In a separate virtual environment, install [`oakd-vio-zmq`](https://github.com/sattwik-sahu/oakd-vio-zmq).
    ```bash
    cd path/to/other/dir

    # Use uv
    uv venv oakd-env
    source oakd-env/bin/activate
    uv pip install git+https://github.com/sattwik-sahu/oakd-vio-zmq.git
    uv pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ --pre -U depthai
    # ... or use default Python tools
    python3 -m venv oakd-env --prompt="oakd-env"
    source oakd-env/bin/activate
    pip install git+https://github.com/sattwik-sahu/oakd-vio-zmq.git
    pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-release-local/ --pre -U depthai
    ```

## Installation

- Clone the repository
    ```bash
    git clone https://github.com/sattwik-sahu/anytraverse_ros.git   # HTTPS
    git clone git@github.com:sattwik-sahu/anytraverse_ros.git   # SSH
    gh repo clone sattwik-sahu/anytraverse_ros   # GH CLI
    ```
- Install the pixi dependencies
    ```bash
    pixi update
    pixi install
    pixi install -e jazzy
    ```
- Build the packages.
    ```bash
    pixi run build
    ```

> [!NOTE]
> Ignore the warnings while building the packages for the first time. It is because the `build`, `install`, and `log` directories have not yet been created.

## Usage

Make sure the OAK-D camera is plugged into the robot, then follow the steps given below.

### Starting the OAK-D Camera

Start the OAK-D camera to get the RGB, depth, and RTABMap localization. Open a terminal and source `oakd-env`, then [start the OAK-D VIO server](https://github.com/sattwik-sahu/oakd-vio-zmq#usage).
```bash
oakd-vio-zmq oakd
```

### Launch the AnyTraverse Navigation Stack

Once the OAK-D camera has been started, run the following commands in a terminal in the `anytraverse-ros` root directory to start the AnyTraverse navigation stack.

1. Start the zenoh router.
    ```bash
    pixi run zenohd
    ```
2. In a different terminal, launch the AnyTraverse navigation script.
    ```bash
    pixi run ros2 launch trav_map_navigation anyt_launch.yaml
    ```

### Foxglove Visualization _(Optional)_

View the topics in Foxglove studio by launching the [ROS Foxglove bridge](https://docs.foxglove.dev/docs/visualization/ros-foxglove-bridge).

```bash
pixi run ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

> [!TIP]
> To view your robot topics on the base station, you must set the `ROS_DOMAIN_ID` on both machines. See [this guide](docs/comm_base.md) for more information.

### Deploying on Real Robots

To see how to deploy on MOON Lab robots, see [this guide](docs/robot.md).

## Contribution

Please feel free to open and issue if you spot a bug or want to request a feature.

---

Made with :heart: at IISER Bhopal
