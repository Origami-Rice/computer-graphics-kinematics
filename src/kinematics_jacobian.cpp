#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  double h = 1e-7;
  Eigen::VectorXd tips = transformed_tips(skeleton, b);
  Eigen::VectorXd t_tips;

  for (int i = 0; i < skeleton.size(); ++i) {
      for (int j = 0; j < 3; ++j) {
          Skeleton copy = skeleton;
          copy[i].xzx[j] += h;
          t_tips = transformed_tips(copy, b);
          for (int k = 0; k < J.rows(); ++k) {
              J(k, 3 * i + j) = (t_tips - tips)[k] / h;
          }
      }
  }
}
