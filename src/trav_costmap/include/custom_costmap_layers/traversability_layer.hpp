#ifndef CUSTOM_COSTMAP_LAYERS__TRAVERSABILITY_LAYER_HPP_
#define CUSTOM_COSTMAP_LAYERS__TRAVERSABILITY_LAYER_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace custom_costmap_layers
{

class TraversabilityLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TraversabilityLayer();
  ~TraversabilityLayer() override;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void matchSize() override;
  void onFootprintChanged() override;

  bool isClearable() override { return clearing_; }

private:
  struct CachedUpdate
  {
    std::unordered_map<unsigned int, double> cell_cost_sum;
    std::unordered_map<unsigned int, int> cell_point_count;
    std::unordered_set<unsigned int> cleared_cells;
  };

  void depthCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & trav_msg,
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg);

  void traceRay(
    const Eigen::Vector3d & origin,
    const Eigen::Vector3d & target,
    double min_z, double max_z,
    std::unordered_set<unsigned int> & cleared_cells,
    unsigned int sx, unsigned int sy, double res, double ox, double oy);

  bool worldToGrid(
    double wx, double wy,
    int & mi, int & mj,
    unsigned int sx, unsigned int sy, double res, double ox, double oy);

  // Parameters
  bool enabled_;
  std::string depth_topic_;
  std::string trav_topic_;
  std::string camera_info_topic_;
  double max_distance_;
  double min_height_;
  double max_height_;
  double trav_threshold_;
  bool clearing_;
  double raytrace_min_range_;
  double raytrace_max_range_;
  int raytrace_stride_;

  // ROS 2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  using ImageSub = message_filters::Subscriber<sensor_msgs::msg::Image>;
  using CameraInfoSub = message_filters::Subscriber<sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<ImageSub> depth_sub_;
  std::shared_ptr<ImageSub> trav_sub_;
  std::shared_ptr<CameraInfoSub> camera_info_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::CameraInfo>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Shared state between callback thread and costmap update thread
  std::mutex data_mutex_;
  std::shared_ptr<CachedUpdate> pending_data_;
  std::atomic<bool> data_ready_{false};
};

}  // namespace custom_costmap_layers

#endif  // CUSTOM_COSTMAP_LAYERS__TRAVERSABILITY_LAYER_HPP_
