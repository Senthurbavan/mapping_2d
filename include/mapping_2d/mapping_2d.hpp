#ifndef MAPPING_2D_HPP
#define MAPPING_2D_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2_ros/create_timer_ros.h"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "mapping_2d/grid3d.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "nav2_util/robot_utils.hpp"
#include <memory>
#include <mutex>

namespace mapping_2d
{

static const char NO_INFORMATION = -1;
static const char OBSTACLE  = 100;
static const char FREE_SPACE  = 0;


class Mapping2D : public nav2_util::LifecycleNode
{
public:

  explicit Mapping2D(
    const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~Mapping2D();

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State &state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State &state) override;

  void calculateMap();
  std::unique_ptr<std::thread> mapProcess;

  void publishMap();
  rclcpp::TimerBase::SharedPtr publish_timer_;

  void octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr &msg);

  bool getRobotPose(geometry_msgs::msg::PoseStamped global_pose) const;

  void mapToWorld(int mx, int my, double& wx, double& wy);
  bool worldToMap(double wx, double wy, int& mx, int& my);
  int getMapIndex(int mx, int my);
  void mapIndexToCells(int index, int&mx, int&my);

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>
    ::SharedPtr map_publisher_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  double update_frequency_;
  int64_t publish_period_ms_;
  std::string global_frame_;      
  std::string robot_base_frame_;  
  double robot_height_;

  std::unique_ptr<octomap::OcTree> tree;

  std::unique_ptr<char[]> mapGrid;

  std::mutex octomap_mutex_;

  double transform_tolerance_;    ///< timeout before transform errors

  // Map params
  double map_origin_x_, map_origin_y_;
  double map_resolution_;
  uint32_t map_size_x_, map_size_y_;
  uint32_t vertical_cells_;
  double obs_th_, fs_th_;

};


}// end of namespace
#endif