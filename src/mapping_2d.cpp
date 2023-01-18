#include "mapping_2d/mapping_2d.hpp"

namespace mapping_2d
{

Mapping2D::Mapping2D(const rclcpp::NodeOptions &options)
  : nav2_util::LifecycleNode("mapping_2d", "", options)
{
  RCLCPP_INFO(get_logger(), "\n -- Creating -- \n");

  declare_parameter("update_frequency", 1.0);
  declare_parameter("publish_frequency", 2.0);
  declare_parameter("robot_base_frame", "base_footprint");
  declare_parameter("global_frame", "map");
}


Mapping2D::~Mapping2D()
{
}

nav2_util::CallbackReturn Mapping2D::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Configuring --\n");

  double publish_frequency;

  get_parameter("update_frequency", update_frequency_);
  get_parameter("publish_frequency", publish_frequency);
  get_parameter("robot_base_frame", robot_base_frame_);
  get_parameter("global_frame", global_frame_);


  publish_period_ms_ = (int64_t)(1000.0/publish_frequency);

  RCLCPP_INFO(get_logger(), "-----Printing Parameters----");
  RCLCPP_INFO(get_logger(), "update_frequency: %f", update_frequency_);
  RCLCPP_INFO(get_logger(), "publish_period: %ldms", publish_period_ms_);
  RCLCPP_INFO(get_logger(), "robot_base_frame: %s", robot_base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "global_frame: %s", global_frame_.c_str());

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "global_map", 10);

  octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "/octomap_full",
    10,
    std::bind(&Mapping2D::octomapCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}


nav2_util::CallbackReturn Mapping2D::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Activating --\n");

  createBond();
  this->map_publisher_->on_activate();

  // First, make sure that the transform between the robot base frame
  // and the global frame is available
  std::string tf_error;

  RCLCPP_INFO(get_logger(), "Checking transform");
  rclcpp::Rate r(2);
  while (rclcpp::ok() & 
    !tf_buffer_->canTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero, &tf_error))
  {
    RCLCPP_INFO(
      get_logger(), "Timed out waiting for transform from %s to %s"
      " to become available, tf error: %s",
      robot_base_frame_.c_str(), global_frame_.c_str(), tf_error.c_str());

    // The error string will accumulate and errors will typically be the same, so the last
    // will do for the warning above. Reset the string here to avoid accumulation
    tf_error.clear();
    r.sleep();
  }

  RCLCPP_INFO(get_logger(), "Transform okay..");

  // mapProcess = new boost::thread(boost::bind(&Mapping2D::calculateMap, this));
  this->mapProcess = std::make_unique<std::thread>(&Mapping2D::calculateMap, this);
  RCLCPP_INFO(get_logger(), "Map process thread is created");

  this->publish_timer_ = create_wall_timer(
    std::chrono::milliseconds(publish_period_ms_),
    std::bind(&Mapping2D::publishMap, this));
  RCLCPP_INFO(get_logger(), "Map publish timer is created");

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Mapping2D::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Deactivating --\n");
  this->publish_timer_->cancel();
  this->mapProcess->join();

  this->map_publisher_->on_deactivate();

  destroyBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Mapping2D::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Cleaning Up --\n");

  this->map_publisher_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

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
  RCLCPP_INFO(get_logger(), "calculateMap thread is running");
  rclcpp::Rate r(update_frequency_);
  int cnt = 0;
  while(rclcpp::ok())
  {
    RCLCPP_INFO(get_logger(), "Thread running #%d", cnt++);
    r.sleep();
  }
}

void Mapping2D::publishMap()
{
  static int cnt = 0;
  RCLCPP_INFO(get_logger(), "Publish Map #%d", cnt++);
}

void Mapping2D::octomapCallback(const octomap_msgs::msg::Octomap::ConstSharedPtr &msg)
{
  static int cnt = 0;
  RCLCPP_INFO(get_logger(), "Receving octomap #%d\n\n", ++cnt);
  tree = std::unique_ptr<octomap::OcTree>((octomap::OcTree*)octomap_msgs::msgToMap(*msg));
}

}// end of namespace

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mapping_2d::Mapping2D)
