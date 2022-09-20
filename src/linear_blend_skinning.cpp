#include "linear_blend_skinning.h"

void linear_blend_skinning(
  const Eigen::MatrixXd & V,
  const Skeleton & skeleton,
  const std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d> > & T,
  const Eigen::MatrixXd & W,
  Eigen::MatrixXd & U)
{
  U.resize(V.rows(), V.cols());
  for (int i = 0; i < V.rows(); ++i) {
      Eigen::Vector3d rest = V.row(i).transpose();
      Eigen::Vector3d base = Eigen::Vector3d::Zero();
      for (int j = 0; j < skeleton.size(); ++j) {
          if (skeleton[j].weight_index == -1) {
              continue;
          }   
          base += W(i, skeleton[j].weight_index) * (T[j].linear() * rest + T[j].translation());
      }
      U.row(i) << base.transpose();
  }
}
