#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  bool loopback = true;
  if (keyframes.size() == 0) {
      return Eigen::Vector3d::Zero();
  }
  if (keyframes.size() == 1) {
      return keyframes.front().second;
  }   
  if (loopback) {
      double interval = keyframes.back().first - keyframes.front().first;
      t = keyframes.front().first + std::fmod(t, interval);
  }
  else {
      if (t <= keyframes.front().first) {
          return keyframes.front().second;
      }      
      if (t >= keyframes.back().first) {
          return keyframes.back().second;
      }          
  }
  double T;
  Eigen::Vector3d theta_0, theta_1, m_0, m_1;
  for (int i = 0; i < keyframes.size(); ++i) {
      auto f1 = keyframes[i];
      if (t > f1.first) {
          continue;
      }
      if (t == f1.first) {
          return f1.second;
      }   
      auto f0 = keyframes[i - 1];
      double t0 = f0.first, t1 = f1.first;
      T = (t - t0) / (t1 - t0);

      auto p0 = f0.second;
      auto p1 = f1.second;
      auto p2 = (i == keyframes.size() - 1) ? f1.second : keyframes[i + 1].second;
      auto p_1 = (i == 1) ? f0.second : keyframes[i - 2].second;

      theta_0 = f0.second;
      theta_1 = f1.second;
      double t_1 = (i == 1) ? f0.first : keyframes[i - 2].first;
      double t2 = (i == keyframes.size() - 1) ? f1.first : keyframes[i + 1].first;

      m_0 = (p1 - p_1) / (t1 - t_1);
      m_1 = (p2 - p0) / (t2 - t0);
      break;
  }
  double T_2 = T * T;
  double T_3 = T_2 * T;
  return (2 * T_3 - 3 * T_2 + 1) * theta_0 + (T_3 - 2 * T_2 + T) * m_0 + (-2 * T_3 + 3 * T_2) * theta_1 + (T_3 - T_2) * m_1;
}
