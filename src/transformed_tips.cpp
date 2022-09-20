#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  Eigen::VectorXd m = Eigen::VectorXd::Zero(3 * b.size());
  std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > T;
  forward_kinematics(skeleton, T);
  for (int i = 0; i < b.size(); ++i) {
      auto tip = T[b[i]] * skeleton[b[i]].rest_T * Eigen::Vector4d(skeleton[b[i]].length, 0.0, 0.0, 1.0);
      m.segment(3 * i, 3) = tip.head(3) / tip[3];
  }
  return m;
}
