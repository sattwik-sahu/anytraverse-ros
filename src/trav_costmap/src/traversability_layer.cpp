#include "custom_costmap_layers/traversability_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <nav2_costmap_2d/cost_values.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  custom_costmap_layers::TraversabilityLayer, nav2_costmap_2d::Layer)

namespace custom_costmap_layers
{

TraversabilityLayer::TraversabilityLayer() = default;
TraversabilityLayer::~TraversabilityLayer() = default;

void TraversabilityLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in TraversabilityLayer::onInitialize");
  }

  // Declare parameters
  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("depth_topic", rclcpp::ParameterValue(std::string("/camera/depth/image_raw")));
  declareParameter("trav_topic", rclcpp::ParameterValue(std::string("/trav_map")));
  declareParameter("camera_info_topic", rclcpp::ParameterValue(std::string("/camera/depth/camera_info")));
  declareParameter("max_distance", rclcpp::ParameterValue(5.0));
  declareParameter("min_height", rclcpp::ParameterValue(0.05));
  declareParameter("max_height", rclcpp::ParameterValue(1.2));
  declareParameter("trav_threshold", rclcpp::ParameterValue(0.5));
  declareParameter("clearing", rclcpp::ParameterValue(true));
  declareParameter("raytrace_min_range", rclcpp::ParameterValue(0.2));
  declareParameter("raytrace_max_range", rclcpp::ParameterValue(5.0));
  declareParameter("raytrace_stride", rclcpp::ParameterValue(8));

  // Read parameters
  enabled_ = node->get_parameter(getFullName("enabled")).as_bool();
  depth_topic_ = node->get_parameter(getFullName("depth_topic")).as_string();
  trav_topic_ = node->get_parameter(getFullName("trav_topic")).as_string();
  camera_info_topic_ = node->get_parameter(getFullName("camera_info_topic")).as_string();
  max_distance_ = node->get_parameter(getFullName("max_distance")).as_double();
  min_height_ = node->get_parameter(getFullName("min_height")).as_double();
  max_height_ = node->get_parameter(getFullName("max_height")).as_double();
  trav_threshold_ = node->get_parameter(getFullName("trav_threshold")).as_double();
  clearing_ = node->get_parameter(getFullName("clearing")).as_bool();
  raytrace_min_range_ = node->get_parameter(getFullName("raytrace_min_range")).as_double();
  raytrace_max_range_ = node->get_parameter(getFullName("raytrace_max_range")).as_double();
  raytrace_stride_ = node->get_parameter(getFullName("raytrace_stride")).as_int();

  RCLCPP_INFO(
    node->get_logger(),
    "TraversabilityLayer initialized: depth=%s, trav=%s, info=%s, "
    "max_dist=%.2f, height=[%.2f, %.2f], trav_thresh=%.2f, clearing=%s, "
    "raytrace=[%.2f, %.2f], stride=%d",
    depth_topic_.c_str(), trav_topic_.c_str(), camera_info_topic_.c_str(),
    max_distance_, min_height_, max_height_, trav_threshold_,
    clearing_ ? "true" : "false",
    raytrace_min_range_, raytrace_max_range_, raytrace_stride_);

  // TF2 - use the buffer already provided by the Layer base class
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Message filter subscribers
  depth_sub_ = std::make_shared<ImageSub>(node, depth_topic_);
  trav_sub_ = std::make_shared<ImageSub>(node, trav_topic_);
  camera_info_sub_ = std::make_shared<CameraInfoSub>(node, camera_info_topic_);

  // Approximate time synchronizer
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *depth_sub_, *trav_sub_, *camera_info_sub_);
  sync_->registerCallback(
    std::bind(&TraversabilityLayer::depthCallback, this, std::placeholders::_1,
              std::placeholders::_2, std::placeholders::_3));

  current_ = true;

  RCLCPP_INFO(node->get_logger(), "TraversabilityLayer subscribers created");
}

void TraversabilityLayer::matchSize()
{
  // Match the internal costmap to the master costmap
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  if (master) {
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(),
      master->getResolution(), master->getOriginX(), master->getOriginY());
  }
}

void TraversabilityLayer::depthCallback(
  const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg,
  const sensor_msgs::msg::Image::ConstSharedPtr & trav_msg,
  const sensor_msgs::msg::CameraInfo::ConstSharedPtr & info_msg)
{
  if (!enabled_) {
    return;
  }

  auto node = node_.lock();
  if (!node) {
    return;
  }

  // === Decode depth image ===
  cv_bridge::CvImageConstPtr depth_cv;
  try {
    depth_cv = cv_bridge::toCvShare(depth_msg, "passthrough");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "Depth cv_bridge error: %s", e.what());
    return;
  }

  // Convert depth to float32 meters
  cv::Mat depth_m;
  if (depth_cv->image.type() == CV_16UC1) {
    depth_cv->image.convertTo(depth_m, CV_32FC1, 1e-3);
  } else if (depth_cv->image.type() == CV_32FC1) {
    depth_m = depth_cv->image;
  } else {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "Unexpected depth image type: %d", depth_cv->image.type());
    return;
  }

  // === Decode traversability map ===
  cv_bridge::CvImageConstPtr trav_cv;
  try {
    trav_cv = cv_bridge::toCvShare(trav_msg, "passthrough");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "Trav cv_bridge error: %s", e.what());
    return;
  }

  cv::Mat trav_float;
  if (trav_cv->image.channels() == 3) {
    cv::Mat channels[3];
    cv::split(trav_cv->image, channels);
    if (trav_cv->image.depth() == CV_8U) {
      channels[0].convertTo(trav_float, CV_32FC1, 1.0 / 255.0);
    } else {
      channels[0].convertTo(trav_float, CV_32FC1);
    }
  } else {
    if (trav_cv->image.depth() == CV_8U) {
      trav_cv->image.convertTo(trav_float, CV_32FC1, 1.0 / 255.0);
    } else {
      trav_cv->image.convertTo(trav_float, CV_32FC1);
    }
  }

  // Ensure dimensions match
  if (depth_m.rows != trav_float.rows || depth_m.cols != trav_float.cols) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "Depth (%dx%d) and trav (%dx%d) dimensions mismatch",
      depth_m.cols, depth_m.rows, trav_float.cols, trav_float.rows);
    return;
  }

  const int h = depth_m.rows;
  const int w = depth_m.cols;

  // === Camera intrinsics ===
  const double fx = info_msg->p[0];
  const double cx = info_msg->p[2];
  const double fy = info_msg->p[5];
  const double cy = info_msg->p[6];

  if (fx <= 0.0 || fy <= 0.0) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "Invalid camera intrinsics: fx=%.2f, fy=%.2f", fx, fy);
    return;
  }

  // === Lookup TF: global frame -> camera optical frame ===
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      layered_costmap_->getGlobalFrameID(), depth_msg->header.frame_id,
      depth_msg->header.stamp, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 5000,
      "TF lookup failed: %s", ex.what());
    return;
  }

  Eigen::Isometry3d T_global_cam = tf2::transformToEigen(tf_msg);

  // Camera origin in global frame
  const Eigen::Vector3d camera_origin_global = T_global_cam * Eigen::Vector3d::Zero();

  // Grid parameters from the costmap
  const unsigned int sx = getSizeInCellsX();
  const unsigned int sy = getSizeInCellsY();
  const double res = getResolution();
  const double ox = getOriginX();
  const double oy = getOriginY();

  if (sx == 0 || sy == 0 || res <= 0.0) {
    return;
  }

  // === Create cached update ===
  auto data = std::make_shared<CachedUpdate>();

  // === RAYTRACING CLEARING (before obstacle writes) ===
  if (clearing_) {
    for (int v = 0; v < h; v += raytrace_stride_) {
      for (int u = 0; u < w; u += raytrace_stride_) {
        float z = depth_m.at<float>(v, u);
        if (std::isnan(z) || z <= 0.0f) {continue;}

        double z_m = static_cast<double>(z);
        if (z_m < raytrace_min_range_ || z_m > raytrace_max_range_) {continue;}

        double Xc = (u - cx) * z_m / fx;
        double Yc = (v - cy) * z_m / fy;

        Eigen::Vector3d point_global = T_global_cam * Eigen::Vector3d(Xc, Yc, z_m);

        traceRay(camera_origin_global, point_global, min_height_, max_height_,
                 data->cleared_cells, sx, sy, res, ox, oy);
      }
    }
  }

  // === OBSTACLE PROCESSING ===
  for (int v = 0; v < h; ++v) {
    for (int u = 0; u < w; ++u) {
      float z = depth_m.at<float>(v, u);
      if (std::isnan(z) || z <= 0.0f) {continue;}

      double z_m = static_cast<double>(z);

      // Range filter
      if (z_m < 0.25 || z_m > max_distance_) {continue;}

      // Traversability filter
      float trav_score = trav_float.at<float>(v, u);
      if (std::isnan(trav_score)) {continue;}
      if (trav_score >= trav_threshold_) {continue;}

      // Unproject to camera optical frame
      double Xc = (u - cx) * z_m / fx;
      double Yc = (v - cy) * z_m / fy;

      // Transform to global frame
      Eigen::Vector3d point_global = T_global_cam * Eigen::Vector3d(Xc, Yc, z_m);

      // Height filter (strictly between min and max)
      if (point_global.z() <= min_height_ || point_global.z() >= max_height_) {
        continue;
      }

      // Project to grid
      int mi, mj;
      if (!worldToGrid(point_global.x(), point_global.y(), mi, mj,
                        sx, sy, res, ox, oy))
      {
        continue;
      }

      unsigned int idx = static_cast<unsigned int>(mi + mj * sx);

      // Cost: linearly map (1.0 - trav_score) to [1, 254]
      double cost_d = (1.0 - static_cast<double>(trav_score)) * 254.0;
      unsigned char cost = static_cast<unsigned char>(
        std::clamp(cost_d, 1.0, 254.0));

      data->cell_cost_sum[idx] += static_cast<double>(cost);
      data->cell_point_count[idx] += 1;
    }
  }

  // === Publish under lock ===
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    pending_data_ = data;
    data_ready_ = true;
  }
}

void TraversabilityLayer::traceRay(
  const Eigen::Vector3d & origin,
  const Eigen::Vector3d & target,
  double min_z, double max_z,
  std::unordered_set<unsigned int> & cleared_cells,
  unsigned int sx, unsigned int sy, double res, double ox, double oy)
{
  Eigen::Vector3d direction = target - origin;
  double ray_length = direction.norm();
  if (ray_length < 1e-6) {return;}

  direction.normalize();

  // Step size: half the costmap resolution
  double step_size = res * 0.5;
  if (step_size <= 0.0) {step_size = 0.05;}
  int num_steps = static_cast<int>(std::ceil(ray_length / step_size));

  for (int s = 1; s < num_steps; ++s) {
    double t = static_cast<double>(s) * step_size;
    if (t >= ray_length) {break;}

    Eigen::Vector3d point = origin + direction * t;

    // Height filter
    if (point.z() <= min_z || point.z() >= max_z) {continue;}

    int mi, mj;
    if (!worldToGrid(point.x(), point.y(), mi, mj, sx, sy, res, ox, oy)) {continue;}

    unsigned int idx = static_cast<unsigned int>(mi + mj * sx);
    cleared_cells.insert(idx);
  }
}

void TraversabilityLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!data_ready_) {return;}

  std::shared_ptr<CachedUpdate> data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = pending_data_;
  }

  if (!data) {return;}

  const double res = getResolution();
  const double ox = getOriginX();
  const double oy = getOriginY();
  const unsigned int sx = getSizeInCellsX();

  // Expand map bounds to cover all affected cells
  auto expand_bounds = [&](unsigned int idx) {
    int i = static_cast<int>(idx % sx);
    int j = static_cast<int>(idx / sx);
    double wx = ox + (static_cast<double>(i) + 0.5) * res;
    double wy = oy + (static_cast<double>(j) + 0.5) * res;
    double half = res * 0.5;
    *min_x = std::min(*min_x, wx - half);
    *min_y = std::min(*min_y, wy - half);
    *max_x = std::max(*max_x, wx + half);
    *max_y = std::max(*max_y, wy + half);
  };

  for (const auto & [idx, _] : data->cell_cost_sum) {
    expand_bounds(idx);
  }
  for (unsigned int idx : data->cleared_cells) {
    expand_bounds(idx);
  }
}

void TraversabilityLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!data_ready_) {return;}

  std::shared_ptr<CachedUpdate> data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    data = pending_data_;
    pending_data_.reset();
    data_ready_ = false;
  }

  if (!data) {return;}

  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();
  const unsigned int total_cells = size_x * size_y;

  // 1) Clearing pass: set FREE_SPACE for raytraced cells
  for (unsigned int idx : data->cleared_cells) {
    if (idx >= total_cells) {continue;}
    int mi = static_cast<int>(idx % size_x);
    int mj = static_cast<int>(idx / size_x);
    master_grid.setCost(mi, mj, nav2_costmap_2d::FREE_SPACE);
  }

  // 2) Obstacle pass: write averaged costs with max-combination
  for (const auto & [idx, cost_sum] : data->cell_cost_sum) {
    if (idx >= total_cells) {continue;}
    int count = data->cell_point_count.at(idx);
    if (count <= 0) {continue;}

    unsigned char avg_cost = static_cast<unsigned char>(
      std::clamp(cost_sum / static_cast<double>(count), 1.0, 253.0));

    int mi = static_cast<int>(idx % size_x);
    int mj = static_cast<int>(idx / size_x);

    // Max-combination: take the higher cost between existing and new
    unsigned char existing = master_grid.getCost(mi, mj);
    master_grid.setCost(mi, mj, std::max(existing, avg_cost));
  }
}

void TraversabilityLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  pending_data_.reset();
  data_ready_ = false;
}

void TraversabilityLayer::onFootprintChanged() {}

bool TraversabilityLayer::worldToGrid(
  double wx, double wy, int & mi, int & mj,
  unsigned int sx, unsigned int sy, double res, double ox, double oy)
{
  mi = static_cast<int>(std::floor((wx - ox) / res));
  mj = static_cast<int>(std::floor((wy - oy) / res));

  if (mi < 0 || mi >= static_cast<int>(sx) ||
      mj < 0 || mj >= static_cast<int>(sy))
  {
    return false;
  }
  return true;
}

}  // namespace custom_costmap_layers
