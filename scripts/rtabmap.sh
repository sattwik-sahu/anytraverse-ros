ros2 launch rtabmap_launch rtabmap.launch.py \
    frame_id:=base_link \
    odom_frame_id:=odom \
    rgb_topic:=/oakd/rgb/image_raw \
    depth_topic:=/oakd/depth/image_raw \
    camera_info_topic:=/oakd/rgb/camera_info \
    approx_sync:=true \
    use_sim_time:=false \
    qos:=2 \
    Vis/MinInliers:=10 \
    Vis/FeatureType:=6 \
    RGBD/OptimizeFromGraphEnd:=false \
    RGBD/NeighborLinkRefining:=true \
    RGBD/ProximityBySpace:=true \
    RGBD/ProximityMaxGraphDepth:=50 \
    RGBD/LoopClosureReextractFeatures:=true \
    RGBD/AngularUpdate:=0.05 \
    RGBD/LinearUpdate:=0.05
