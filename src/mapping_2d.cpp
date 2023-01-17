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

  // mapProcess = new boost::thread(boost::bind(&Mapping2D::calculateMap, this));
  this->mapProcess = std::make_unique<std::thread>(&Mapping2D::calculateMap, this);
  RCLCPP_INFO(get_logger(), "Map process thread is created");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Mapping2D::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Deactivating --\n");
  mapProcess->join();

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

void Mapping2D::calculateMap()
{
  RCLCPP_INFO(get_logger(), "\n\ncalculateMap thread is running \n\n");
  rclcpp::Rate r(1);
  int cnt = 0;
  while(rclcpp::ok())
  {
    RCLCPP_INFO(get_logger(), "\n\nThread running #%d\n\n", cnt++);
    r.sleep();
  }
}



}// end of namespace

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mapping_2d::Mapping2D)
