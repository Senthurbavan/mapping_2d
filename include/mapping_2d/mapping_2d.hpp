#ifndef MAPPING_2D_HPP
#define MAPPING_2D_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/time.h"
#include "tf2_ros/create_timer_ros.h"
#include "mapping_2d/grid3d.hpp"



namespace mapping_2d
{

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

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  // Transform listener
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  double update_frequency_;
  int64_t publish_period_ms_;
  std::string global_frame_;      
  std::string robot_base_frame_;  

};


}// end of namespace
#endif