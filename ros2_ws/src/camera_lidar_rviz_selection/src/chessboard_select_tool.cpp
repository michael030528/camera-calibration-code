#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <QString>
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/selection_manager_iface.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction_iface.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_default_plugins/tools/select/selection_tool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"

namespace camera_lidar_rviz_selection
{

class ChessboardSelectTool : public rviz_default_plugins::tools::SelectionTool
{
public:
  void onInitialize() override
  {
    rviz_default_plugins::tools::SelectionTool::onInitialize();
    auto abstraction = context_->getRosNodeAbstraction().lock();
    if (!abstraction) {
      throw std::runtime_error("RViz ROS node is unavailable");
    }
    node_ = abstraction->get_raw_node();
    freeze_pub_ = node_->create_publisher<std_msgs::msg::Empty>(
      "/calibration/freeze_selection", rclcpp::QoS(1).reliable());
    selected_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/calibration/selected_points", rclcpp::SensorDataQoS());
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/calibration/selection_cloud", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(cloud_mutex_);
        latest_cloud_ = std::move(msg);
      });
    setName("Chessboard Select");
    setDescription(
      "Drag a rectangle around chessboard points. The synchronized frame is frozen "
      "on mouse-down and the selected points are published on mouse-up.");
  }

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override
  {
    if (event.leftDown()) {
      freeze_pub_->publish(std_msgs::msg::Empty());
      selecting_board_ = true;
      setStatus("Synchronized frame requested; drag around the chessboard points");
    }

    const int result = rviz_default_plugins::tools::SelectionTool::processMouseEvent(event);

    if (event.leftUp() && selecting_board_) {
      selecting_board_ = false;
      publishSelectedPoints();
    }
    return result;
  }

private:
  void publishSelectedPoints()
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      cloud = latest_cloud_;
    }
    if (!cloud) {
      setStatus("No synchronized point cloud is available");
      return;
    }

    std::set<uint64_t> indices;
    const auto & selection = context_->getSelectionManager()->getSelection();
    const uint64_t point_count =
      static_cast<uint64_t>(cloud->width) * static_cast<uint64_t>(cloud->height);
    for (const auto & entry : selection) {
      for (const uint64_t handle : entry.second.extra_handles) {
        const uint64_t encoded_index = handle & 0xffffffffULL;
        if (encoded_index == 0) {
          continue;
        }
        const uint64_t index = encoded_index - 1;
        if (index < point_count) {
          indices.insert(index);
        }
      }
    }

    if (indices.empty()) {
      setStatus("No point-cloud points selected; drag tightly around the board");
      return;
    }
    if (cloud->width == 0 || cloud->point_step == 0) {
      setStatus("The synchronized point cloud has an invalid layout");
      return;
    }

    auto out = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out->header = cloud->header;
    out->height = 1;
    out->width = static_cast<uint32_t>(indices.size());
    out->fields = cloud->fields;
    out->is_bigendian = cloud->is_bigendian;
    out->point_step = cloud->point_step;
    out->row_step = out->point_step * out->width;
    out->is_dense = cloud->is_dense;
    out->data.resize(out->row_step);

    size_t destination_offset = 0;
    for (const uint64_t index : indices) {
      const uint64_t row = index / cloud->width;
      const uint64_t column = index % cloud->width;
      const uint64_t source_offset = row * cloud->row_step + column * cloud->point_step;
      if (source_offset + cloud->point_step > cloud->data.size()) {
        continue;
      }
      std::memcpy(
        out->data.data() + destination_offset,
        cloud->data.data() + source_offset,
        cloud->point_step);
      destination_offset += cloud->point_step;
    }
    out->width = static_cast<uint32_t>(destination_offset / out->point_step);
    out->row_step = out->width * out->point_step;
    out->data.resize(out->row_step);

    const auto count = out->width;
    selected_pub_->publish(std::move(out));
    setStatus(QString("Published %1 selected chessboard points").arg(count));
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr freeze_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr selected_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_cloud_;
  std::mutex cloud_mutex_;
  bool selecting_board_{false};
};

}  // namespace camera_lidar_rviz_selection

PLUGINLIB_EXPORT_CLASS(
  camera_lidar_rviz_selection::ChessboardSelectTool,
  rviz_common::Tool)
