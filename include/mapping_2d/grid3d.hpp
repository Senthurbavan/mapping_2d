#ifndef MAPPING_GRID3D_HPP
#define MAPPING_GRID3D_HPP

#include <rclcpp/rclcpp.hpp>
#include "nav2_util/lifecycle_node.hpp"

namespace mapping_2d
{

enum CellStatus {
  FREE = 0,
  UNKNOWN = 1,
  MARKED = 2,
};

class Grid3D
{
public:

  Grid3D(const nav2_util::LifecycleNode::WeakPtr &parent);
  Grid3D(const nav2_util::LifecycleNode::WeakPtr &parent,
          double lx, double ly, double lz,
          double orix, double oriy, double oriz,
          double res, int mkth, int unth);
  ~Grid3D();
  bool validCell(int xm, int ym, int zm);
  int getIndex(int xm, int ym, int zm);
  unsigned char getCell(int xm, int ym, int zm);
  bool markCell(int xm, int ym, int zm);
  bool clearCell(int xm, int ym, int zm);
  unsigned char getColumnCost(int xm, int ym);
  void gridToWorld(int xm, int ym, int zm, double& wx, double& wy, double& wz);
  bool worldToGrid(double wx, double wy, double wz, int& xm, int& ym, int& zm);
  int sizeX(void){return size_x_;};
  int sizeY(void){return size_y_;};
  int sizeZ(void){return size_z_;};
  int size(void){return size_x_*size_y_*size_z_;};

private:
  rclcpp::Logger logger_{rclcpp::get_logger("mapping_2d")};

  void initGrid(void);

  unsigned char* grid;
  double length_x_, length_y_, length_z_;
  double origin_x_, origin_y_, origin_z_; 
  int size_x_, size_y_, size_z_;
  double resolution_;
  int mark_th_, unknown_th_; 
};

}

#endif