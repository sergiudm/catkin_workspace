#include <Eigen/Core>
#include <Eigen/Dense>
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
  T(1, 3) = -sin(alpha) * d;

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
  Eigen::Vector4d l0 = {0, 0, 0.1283 + 0.115, thetas[0]};

  Eigen::Vector4d l1 = {M_PI / 2.0, 0, 0.030, thetas[1] + M_PI / 2.0};

  Eigen::Vector4d l2 = {M_PI, 0.280, 0.020, thetas[2] + M_PI / 2.0};

  Eigen::Vector4d l3 = {M_PI / 2.0, 0, 0.140 + 0.105, thetas[3] + M_PI / 2.0};

  Eigen::Vector4d l4 = {M_PI / 2.0, 0, 0.0285 + 0.0285, thetas[4] + M_PI};

  Eigen::Vector4d l5 = {M_PI / 2.0, 0, 0.105 + 0.130, thetas[5] + M_PI};

  std::vector<Eigen::Vector4d> DH_tables = {l0, l1, l2, l3, l4, l5};

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
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
  result(5) = atan2(T(1, 0), T(0, 0)); // roll
  result(4) =
      atan2(-T(2, 0), sqrt(T(2, 1) * T(2, 1) + T(2, 2) * T(2, 2))); // pitch
  result(3) = atan2(T(2, 1), T(2, 2));                              // yaw

  for (int i = 0; i < 3; ++i) {
    result[i] *= 1000;
  }

  for (int i = 3; i < 6; ++i) {
    result[i] *= 180 / M_PI;
  }

  return result;
}

int main() {
  Eigen::VectorXd test1(6);
  test1 << 0, 345, 75, 0, 300, 0;
  Eigen::VectorXd test2(6);
  test2 << 0, 0, 0, 0, 0, 0;
  Eigen::VectorXd test3(6);
  test3 << 357, 21, 150, 272, 320, 273;
  Eigen::VectorXd test4(6);
  test4 << 270, 148, 148, 270, 140, 0;
  Eigen::VectorXd test5(6);
  test5 << 20.5, 313.5, 100, 265.5, 327, 57;

  auto t1 = T_base2tool(test1);
  auto t2 = T_base2tool(test2);
  auto t3 = T_base2tool(test3);
  auto t4 = T_base2tool(test4);
  auto t5 = T_base2tool(test5);

  auto p1 = transformation(test1);
  auto p2 = transformation(test2);
  auto p3 = transformation(test3);
  auto p4 = transformation(test4);
  auto p5 = transformation(test5);

  std::cout << "t1: " << t1 << std::endl;
  std::cout << "t2: " << t2 << std::endl;
  std::cout << "t3: " << t3 << std::endl;
  std::cout << "t4: " << t4 << std::endl;
  std::cout << "t5: " << t5 << std::endl;

  std::cout << "p1: " << p1.transpose() << std::endl;
  std::cout << "p2: " << p2.transpose() << std::endl;
  std::cout << "p3: " << p3.transpose() << std::endl;
  std::cout << "p4: " << p4.transpose() << std::endl;
  std::cout << "p5: " << p5.transpose() << std::endl;
}