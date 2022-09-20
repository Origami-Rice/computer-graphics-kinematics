#include "euler_angles_to_transform.h"
#include <corecrt_math_defines.h>

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  Eigen::AngleAxis<double> x1, x2, z;
  double t1 = xzx[0] * M_PI / 180.0;
  double t2 = xzx[1] * M_PI / 180.0;
  double t3 = xzx[2] * M_PI / 180.0;
  x1 = Eigen::AngleAxis<double>(t1, Eigen::Vector3d::UnitX());
  z = Eigen::AngleAxis<double>(t2, Eigen::Vector3d::UnitZ());
  x2 = Eigen::AngleAxis<double>(t3, Eigen::Vector3d::UnitX());

  return Eigen::Affine3d(x2 * z * x1);
}
