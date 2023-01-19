#include "mapping_2d/mapping_2d.hpp"

namespace mapping_2d
{

Mapping2D::Mapping2D(const rclcpp::NodeOptions &options)
  : nav2_util::LifecycleNode("mapping_2d", "", options)
{
  RCLCPP_INFO(get_logger(), "\n -- Creating -- \n");

  declare_parameter("update_frequency", 1.0);
  declare_parameter("publish_frequency", 2.0);
  declare_parameter("transform_tolerance", 0.2);
  declare_parameter("robot_base_frame", "base_footprint");
  declare_parameter("global_frame", "map");
  declare_parameter("robot_height", 0.5);

  declare_parameter("map_origin_x", 0.0);
  declare_parameter("map_origin_y", 0.0);
  declare_parameter("map_length_x", 10.0); // in meters
  declare_parameter("map_length_y", 10.0); // in meters
  declare_parameter("obstacle_threshold", 0.1);
  declare_parameter("free_space_threshold", 0.9);
}


Mapping2D::~Mapping2D()
{
}

nav2_util::CallbackReturn Mapping2D::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "\n -- Configuring --\n");

  double publish_frequency, map_length_x, map_length_y;

  get_parameter("update_frequency", update_frequency_);
  get_parameter("publish_frequency", publish_frequency);
  get_parameter("transform_tolerance", transform_tolerance_);
  get_parameter("robot_base_frame", robot_base_frame_);
  get_parameter("global_frame", global_frame_);
  get_parameter("robot_height", robot_height_);

  get_parameter("map_resolution", map_resolution_);
  get_parameter("map_origin_x", map_origin_x_);
  get_parameter("map_origin_y", map_origin_y_);
  get_parameter("map_length_x", map_length_x);
  get_parameter("map_length_y", map_length_y);
  get_parameter("obstacle_threshold", obs_th_);
  get_parameter("free_space_threshold", fs_th_);

  publish_period_ms_ = (int64_t)(1000.0/publish_frequency);
  map_size_x_ = (uint32_t)(map_length_x/map_resolution_);
  map_size_y_ = (uint32_t)(map_length_y/map_resolution_);
  vertical_cells_ = (uint32_t)((robot_height_ - 1.5*map_resolution_)/map_resolution_);

  RCLCPP_INFO(get_logger(), "-----Printing Parameters----");
  RCLCPP_INFO(get_logger(), "update_frequency: %f", update_frequency_);
  RCLCPP_INFO(get_logger(), "publish_period: %ldms", publish_period_ms_);
  RCLCPP_INFO(get_logger(), "transform_tolerance: %f", transform_tolerance_);
  RCLCPP_INFO(get_logger(), "robot_base_frame: %s", robot_base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "global_frame: %s", global_frame_.c_str());
  RCLCPP_INFO(get_logger(), "robot height: %f", robot_height_);

  RCLCPP_INFO(get_logger(), "map resolution: %f", map_resolution_);
  RCLCPP_INFO(get_logger(), "map origin x: %f", map_origin_x_);
  RCLCPP_INFO(get_logger(), "map origin y: %f", map_origin_y_);
  RCLCPP_INFO(get_logger(), "map size x: %d (%fm)", map_size_x_, map_length_x);
  RCLCPP_INFO(get_logger(), "map size y: %d (%fm)", map_size_y_, map_length_y);
  RCLCPP_INFO(get_logger(), "obstacle_threshold: %f", obs_th_);
  RCLCPP_INFO(get_logger(), "free_space_threshold: %f", fs_th_);

  // Create the transform-related objects
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface(),
    callback_group_);
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  mapGrid = std::unique_ptr<char[]>(new char[map_size_x_*map_size_y_]);
  memset(mapGrid.get(), NO_INFORMATION, map_size_x_*map_size_y_*sizeof(char));
  RCLCPP_INFO(get_logger(), "map chack: %d, %d, %d", mapGrid[0], 
    mapGrid[map_size_x_*map_size_y_*sizeof(char)/2], 
    mapGrid[map_size_x_*map_size_y_*sizeof(char)]);

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

    geometry_msgs::msg::PoseStamped robot_pose;
    
    if(getRobotPose(robot_pose))
    {
      double lower = robot_pose.pose.position.z + 1.5 * map_resolution_;
      double upper = robot_pose.pose.position.z + robot_height_;

      std::unique_lock<std::mutex> octomap_lock(octomap_mutex_);

      for (int xi = 0; xi < (int)map_size_x_; xi++)
      {
        for (int yi = 0; yi < (int)map_size_y_; yi++)
        {
          int obs_cells = 0;
          int fs_cells = 0;
          for (double zf = lower; zf < upper; zf += map_resolution_)
          {
            double wx, wy;
            mapToWorld(xi, yi, wx, wy);
            octomap::OcTreeNode *node;
            if ((node = tree->search(wx, wy, zf)))
            {
              if (tree->isNodeOccupied(node))
              {
                obs_cells++;
              }else
              {
                fs_cells++;
              }
            }
          }

          int id = getMapIndex(xi, yi);

          if (obs_cells >= vertical_cells_*obs_th_)
          {
            mapGrid[id] = OBSTACLE;
          }else if (fs_cells >= vertical_cells_ * fs_th_)
          {
            mapGrid[id] = FREE_SPACE;
          }else
          {
            mapGrid[id] = NO_INFORMATION;
          }
        }
      }
        
      octomap_lock.unlock();

      //publish gridmap

    }
    else{
      RCLCPP_INFO(get_logger(), "Could not get current robot pose!!");
    }

    
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

  std::unique_lock<std::mutex> lock(octomap_mutex_);
  
  tree = std::unique_ptr<octomap::OcTree>((octomap::OcTree*)octomap_msgs::msgToMap(*msg));
  
  lock.unlock();
}

bool Mapping2D::getRobotPose(geometry_msgs::msg::PoseStamped global_pose) const
{
  return nav2_util::getCurrentPose(
    global_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_);  
}

void Mapping2D::mapToWorld(int mx, int my, double& wx, double& wy)
{
	wx = map_origin_x_ + (mx + 0.5)*map_resolution_;
	wy = map_origin_y_ + (my + 0.5)*map_resolution_;
}

bool Mapping2D::worldToMap(double wx, double wy, int& mx, int& my)
{
	mx = (int)((wx - map_origin_x_) / map_resolution_);
	my = (int)((wy - map_origin_y_) / map_resolution_);

	if (mx < (int)map_size_x_ && my < (int)map_size_y_ && wx >= map_origin_x_ &&  wy >= map_origin_y_)
	{
		return true;
	}

	return false;
}

int Mapping2D::getMapIndex(int mx, int my)
{
    return my*map_size_x_ + mx;
}

void Mapping2D::mapIndexToCells(int index, int&mx, int&my)
{
	my = index/map_size_x_;
	mx = index - (my*map_size_x_);
}


}// end of namespace

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mapping_2d::Mapping2D)
