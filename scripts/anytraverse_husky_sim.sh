#!/usr/bin/bash

pixi run -e jazzy ros2 launch anytraverse_bringup anytraverse_sim.launch.py \
    use_sim_time:=true \
    nav_params_file:=./scripts/husky_nav2_params.yaml \
    init_prompt:="grass: 1.0" \
    robot:=a200_0000 \
    camera_rgb_topic:=/a200_0000/sensors/camera_0/color/image \
    camera_rgb_info_topic:=/a200_0000/sensors/camera_0/color/camera_info \
    camera_depth_topic:=/a200_0000/sensors/camera_0/depth/image \
    camera_depth_info_topic:=/a200_0000/sensors/camera_0/depth/camera_info \
    camera_optical_frame:=camera_0_color_optical_frame
