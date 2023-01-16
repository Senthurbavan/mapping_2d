#include "mapping_2d/mapping_2d.hpp"

namespace mapping_2d
{

Mapping2D::Mapping2D(const rclcpp::NodeOptions &options)
  : nav2_util::LifecycleNode("mapping_2d", "", options)
{
  RCLCPP_INFO(get_logger(), "\n -- Creating -- \n");
}


Mapping2D::~Mapping2D()
{
}

nav2_util::CallbackReturn Mapping2D::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Configuring --\n");

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn Mapping2D::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Activating --\n");

  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Mapping2D::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Deactivating --\n");


  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Mapping2D::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Cleaning Up --\n");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Mapping2D::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Shutting Down --\n");

  return nav2_util::CallbackReturn::SUCCESS;
}
}// end of namespace

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mapping_2d::Mapping2D)
