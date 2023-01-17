#include "mapping_2d/grid3d.hpp"

namespace mapping_2d
{

Grid3D::Grid3D(const nav2_util::LifecycleNode::WeakPtr &parent)
  :length_x_(0),length_y_(0),length_z_(0),
    origin_x_(0),origin_y_(0),origin_z_(0),
    resolution_(0.05), mark_th_(1), unknown_th_(-1)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  initGrid();
}


Grid3D::Grid3D(const nav2_util::LifecycleNode::WeakPtr &parent,
  double lx, double ly, double lz,
  double orix, double oriy, double oriz,
  double res, int mkth, int unth)
  :length_x_(lx),length_y_(ly),length_z_(lz),
    origin_x_(orix),origin_y_(oriy),origin_z_(oriz),
    resolution_(res), mark_th_(mkth), unknown_th_(unth)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  initGrid();
}


Grid3D::~Grid3D()
{
  delete[] grid; 
}


void Grid3D::initGrid()
{
  size_x_ = int(length_x_/resolution_);
  size_y_ = int(length_y_/resolution_);
  size_z_ = int(length_z_/resolution_);

  if(unknown_th_ == -1)
  {
    unknown_th_ = (int)(0.95*size_z_);
  }
  
  RCLCPP_INFO(logger_, "3D grid initialized with x_size: %d," 
                        "y_size: %d, z_size: %d, unknown_th: %d",
                        size_x_, size_y_, size_z_,unknown_th_);

  int size = size_x_*size_y_*size_z_;
  grid = new unsigned char[size];
  for (int i = 0; i < size; i++)
  {
    grid[i] = UNKNOWN;
  }
}


bool Grid3D::validCell(int xm, int ym, int zm)
{
  if ((xm>=size_x_)||(ym>=size_y_)||(zm>=size_z_))
  {
    return false;
  }
  if ((xm < 0)||(ym < 0)||(zm < 0))
  {
    return false;
  }
  return true;
}


int Grid3D::getIndex(int xm, int ym, int zm)
{
  return xm*size_z_ + ym*size_x_*size_z_ + zm;
}


unsigned char Grid3D::getCell(int xm, int ym, int zm)
{
  if (!validCell(xm, ym, zm))
  {
    RCLCPP_INFO(logger_, "getCell: cell [%d, %d, %d] is not valid", xm, ym, zm);
    return UNKNOWN;
  }
  return grid[getIndex(xm, ym, zm)];
}


bool Grid3D::markCell(int xm, int ym, int zm)
{
  if (!validCell(xm, ym, zm))
  {
    RCLCPP_INFO(logger_, "markCell: cell [%d, %d, %d] is not valid", xm, ym, zm);
    return false;
  }
  grid[getIndex(xm, ym, zm)] = MARKED;
  return true;
}


bool Grid3D::clearCell(int xm, int ym, int zm)
{
  if (!validCell(xm, ym, zm))
  {
    RCLCPP_INFO(logger_, "clearCell: cell [%d, %d, %d] is not valid", xm, ym, zm);
    return false;
  }
  grid[getIndex(xm, ym, zm)] = FREE;
  return true;
}


unsigned char Grid3D::getColumnCost(int xm, int ym)
{
  if ((xm<0)||(xm>=size_x_)||(ym<0)||(ym>=size_y_))
  {
    RCLCPP_INFO(logger_, "getColumn: cell [%d, %d] is not valid", xm, ym);
    return UNKNOWN;
  }
  
  int mark_cells = 0;
  int unknown_cells = 0;

  for (int zi = 0; zi < size_z_; zi++)
  {
    unsigned char val = grid[xm*size_z_ + ym*size_x_*size_z_ + zi];
    if (val == MARKED)
    {
      mark_cells++;
    }else if (val == UNKNOWN)
    {
      unknown_cells++;
    }

    if (mark_cells >= mark_th_)
    {
      return MARKED;
    }
  }

  if (unknown_cells >= unknown_th_)
  {
    return UNKNOWN;
  }

  return FREE;
}


void Grid3D::gridToWorld(int xm, int ym, int zm, double& wx, double& wy, double& wz)
{
  wx = origin_x_ + (xm + 0.5)*resolution_;
	wy = origin_y_ + (ym + 0.5)*resolution_;
  wz = origin_z_ + (zm + 0.5)*resolution_;
}


bool Grid3D::worldToGrid(double wx, double wy, double wz, int& xm, int& ym, int& zm)
{
  if (wx < origin_x_ || wy < origin_y_ || wz < origin_z_) 
	{
		return false;
	}

	xm = (int)((wx - origin_x_) / resolution_);
	ym = (int)((wy - origin_y_) / resolution_);
  zm = (int)((wz - origin_z_) / resolution_);

	if (xm < size_x_ && ym < size_y_ && zm < size_z_)
	{
		return true;
	}

	return false;
}

}