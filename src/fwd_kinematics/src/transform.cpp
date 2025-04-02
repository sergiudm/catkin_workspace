#include "Eigen/Core"
#include "Eigen/Dense"
#include <cmath>

Eigen::Matrix4d transform(Eigen::Vector4d DH_table) {
  Eigen::Matrix4d T;
  T.setIdentity();
  double alpha = DH_table[0];
  double a = DH_table[1];
  double d = DH_table[2];
  double theta = DH_table[3];

  T(0, 0) = cos(theta);
  T(0, 1) = -sin(theta);
  T(0, 2) = 0;
  T(0, 3) = a;

  T(1, 0) = sin(theta) * cos(alpha);
  T(1, 1) = cos(theta) * cos(alpha);
  T(1, 2) = -sin(alpha);
  T(1, 3) = -1 * sin(alpha) * d;

  T(2, 0) = sin(theta) * sin(alpha);
  T(2, 1) = cos(theta) * sin(alpha);
  T(2, 2) = cos(alpha);
  T(2, 3) = cos(alpha) * d;

  T(3, 0) = 0;
  T(3, 1) = 0;
  T(3, 2) = 0;
  T(3, 3) = 1;

  return T;
}

Eigen::Matrix4d T_base2tool(Eigen::Vector4d thetas) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Vector4d l1 = {90, 0, 128.3 + 115.0, thetas[0]};
  Eigen::Vector4d l2 = {180, 280, 30, thetas[1] + 90};
  Eigen::Vector4d l3 = {90, 0, 20, thetas[2] + 90};
  Eigen::Vector4d l4 = {90, 0, 140.0 + 105.0, thetas[3] + 90};
  Eigen::Vector4d l5 = {90, 0, 28.5 + 28.5, thetas[4] + 180};
  Eigen::Vector4d l6 = {0, 0, 105.0 + 130.0, thetas[5] + 90};

  std::vector<Eigen::Vector4d> DH_tables = {l1, l2, l3, l4, l5, l6};
  for (int i = 0; i < DH_tables.size(); i++) {
    T = T * transform(DH_tables[i]);
  }
  return T;
}

Eigen::VectorXd transformation(Eigen::Vector4d thetas) {
  Eigen::Matrix4d T = T_base2tool(thetas);
  Eigen::VectorXd result(6);
  result(0) = T(0, 3);                            // x
  result(1) = T(1, 3);                            // y
  result(2) = T(2, 3);                            // z
  result(3) = atan2(T(1, 0), T(0, 0)) * 180 / PI; // roll
  result(4) = atan2(-T(2, 0), sqrt(T(2, 1) * T(2, 1) + T(2, 2) * T(2, 2))) *
              180 / PI;                           // pitch
  result(5) = atan2(T(2, 1), T(2, 2)) * 180 / PI; // yaw
  // Convert radians to degrees
  return result;
}

int main() {
    Eigen::Vector4d test1 = {0, 0, 0, 0};
    Eigen::Vector4d test2 = {0, 0, 0, 90};
}