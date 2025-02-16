// main.cpp
#include <iostream>
#include <iomanip>
#include <chrono>
#include "rigidbody.h"

#include <Eigen/Dense>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/crba.hpp"

Eigen::VectorXd cast_farray(float* arr) {
  Eigen::Map<Eigen::VectorXf> vec_float(arr, 7); 
  Eigen::VectorXd vec = vec_float.cast<double>();
  return vec;
}

Eigen::VectorXd cast_darray(double* arr) {
  Eigen::Map<Eigen::VectorXd> vec(arr, 7);
  return vec;
}

auto benchmark(const std::string& name, const std::function<void()>& func) {
  auto start = std::chrono::high_resolution_clock::now();
  func();
  auto end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::micro> duration = end - start;
  std::cout << name << " execution time: " << duration.count() << " microseconds" << std::endl;
}


void bench_pinocchio(double* q, double* dq, double* ddq) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel("assets/fr3.urdf", model);
  pinocchio::Data data(model);

  Eigen::VectorXd q_ = cast_darray(q);
  Eigen::VectorXd dq_ = cast_darray(dq);
  Eigen::VectorXd ddq_ = cast_darray(ddq);
  
  benchmark("pinocchio RNEA", [&]() { pinocchio::rnea(model, data, q_, dq_, ddq_); });
  pinocchio::rnea(model, data, q_, dq_, ddq_);
  auto pin_tau = data.tau;
  std::cout << "pinocchio: " << pin_tau.transpose() << std::endl;

  benchmark("pinocchio fwd_kin", [&]() { pinocchio::forwardKinematics(model, data, q_); });
  pinocchio::forwardKinematics(model, data, q_);
  Eigen::VectorXd pin_pose = data.oMi[6].translation();
  std::cout << "pinocchio: " << pin_pose.transpose() << std::endl;

  benchmark("pinocchio crba", [&]() { pinocchio::forwardKinematics(model, data, q_); });
  Eigen::MatrixXd pin_H = pinocchio::crba(model, data, q_); 
  std::cout << "pinocchio: " << pin_H << std::endl;
}

void bench_rigidbody(double* q, double* dq, double* ddq) {
  Multibody* mb = multibody_new();

  benchmark("rigidbody RNEA", [&]() {multibody_rnea(mb, q, dq, ddq); });
  double* tau = multibody_rnea(mb, q, dq, ddq);
  std::cout << "rigidbody: " << cast_darray(tau).transpose() << std::endl;

  benchmark("rigidbody fwd_kin", [&]() {multibody_fwd_kin(mb, q); });
  double* pos = multibody_fwd_kin(mb, q);
  Eigen::Map<Eigen::VectorXd> vec(pos, 3);
  std::cout << "rigidbody: " << vec.transpose() << std::endl;

  benchmark("rigidbody crba", [&]() {multibody_crba(mb, q); });
  double* H = multibody_crba(mb, q);
  std::cout << "rigidbody: ";
  Eigen::MatrixXd H_;
  for (int i = 0; i<7; i++) {
    for (int j = 0; j<7; j++) {
      H_(i, j) = H[i+7*j];
    }
  }
  std::cout << "rigidbody: " << H_ << std::endl;
}

  
int main() {
  double q[7]   = {0.0f, 1.0f, 1.0f, .0f, 0.0f, 0.0f, 0.0f};
  double dq[7]  = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  double ddq[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};

  std::cout << "  q:" << cast_darray(q).transpose() << std::endl;
  std::cout << " dq:" << cast_darray(dq).transpose() << std::endl;
  std::cout << "ddq:" << cast_darray(ddq).transpose() << std::endl;

  bench_pinocchio(q, dq, ddq);
  bench_rigidbody(q, dq, ddq);

  return 0;
}
