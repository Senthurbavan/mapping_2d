#ifndef MAPPING_2D_HPP
#define MAPPING_2D_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"
// #include <boost/thread.hpp>
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


  double update_frequency_;
  int64_t publish_period_ms_;

};


}// end of namespace
#endif