#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>
#include <iostream>
#include <vector>

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

Eigen::Matrix4d T_base2tool(Eigen::VectorXd thetas) {
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  Eigen::Vector4d l1 = {M_PI / 2.0, 0, 128.3 + 115.0, thetas[0]};
  Eigen::Vector4d l2 = {M_PI, 280, 30, thetas[1] + M_PI / 2.0};
  Eigen::Vector4d l3 = {M_PI / 2.0, 0, 20, thetas[2] + M_PI / 2.0};
  Eigen::Vector4d l4 = {M_PI / 2.0, 0, 140.0 + 105.0, thetas[3] + M_PI / 2.0};
  Eigen::Vector4d l5 = {M_PI / 2.0, 0, 28.5 + 28.5, thetas[4] + M_PI};
  Eigen::Vector4d l6 = {0, 0, 105.0 + 130.0, thetas[5] + M_PI / 2.0};

  std::vector<Eigen::Vector4d> DH_tables = {l1, l2, l3, l4, l5, l6};
  for (int i = 0; i < DH_tables.size(); i++) {
    T = T * transform(DH_tables[i]);
  }
  return T;
}

Eigen::VectorXd transformation(Eigen::VectorXd thetas) {
  // convert thetas to radians
  for (int i = 0; i < thetas.size(); i++) {
    thetas[i] = thetas[i] * M_PI / 180.0;
  }
  Eigen::Matrix4d T = T_base2tool(thetas);
  Eigen::VectorXd result(6);
  result(0) = T(0, 3);                 // x
  result(1) = T(1, 3);                 // y
  result(2) = T(2, 3);                 // z
  result(3) = atan2(T(1, 0), T(0, 0)); // roll
  result(4) =
      atan2(-T(2, 0), sqrt(T(2, 1) * T(2, 1) + T(2, 2) * T(2, 2))); // pitch
  result(5) = atan2(T(2, 1), T(2, 2));                              // yaw
  // Convert radians to degrees
  return result;
}

int main() {
  Eigen::VectorXd test1(6);
  test1 << 0, 345, 75, 0, 300, 0;
  Eigen::VectorXd test2(6);
  test2 << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd test3(6);
  test3 << 357, 21, 150, 272, 320, 273;

  auto T1 = transformation(test1);
  auto T2 = transformation(test2);
  auto T3 = transformation(test3);
  std::cout << "T1: " << T1.transpose() << std::endl;
  std::cout << "T2: " << T2.transpose() << std::endl;
  std::cout << "T3: " << T3.transpose() << std::endl;
}