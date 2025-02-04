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


Eigen::VectorXd cast_farray(float* arr) {
  Eigen::Map<Eigen::VectorXf> vec_float(arr, 7); 
  Eigen::VectorXd vec = vec_float.cast<double>();
  return vec;
}


Eigen::VectorXd cast_darray(double* arr) {
  Eigen::Map<Eigen::VectorXd> vec(arr, 7);
  return vec;
}


int main() {
  Multibody* mb = multibody_new();

  pinocchio::Model model;
  pinocchio::urdf::buildModel("assets/fr3.urdf", model);
  pinocchio::Data data(model);
  
  double q[7]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  double dq[7]  = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  double ddq[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};

  Eigen::VectorXd q_ = cast_darray(q);
  Eigen::VectorXd dq_ = cast_darray(dq);
  Eigen::VectorXd ddq_ = cast_darray(ddq);
    
  std::cout << "  q:" << q_.transpose() << std::endl;
  std::cout << " dq:" << dq_.transpose() << std::endl;
  std::cout << "ddq:" << ddq_.transpose() << std::endl;


  auto start_rnea = std::chrono::high_resolution_clock::now();
  pinocchio::rnea(model, data, q_, dq_, ddq_);
  Eigen::VectorXd pin_tau = data.tau;    
  auto end_rnea = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> duration_rnea = end_rnea - start_rnea;

  std::cout << "pinocchio:    " << pin_tau.transpose() << std::endl;
  std::cout << "runRNEA execution time: " << duration_rnea.count() << " microseconds" << std::endl;

  // Timing for multibody_rnea
  auto start_mb_rnea = std::chrono::high_resolution_clock::now();
  double* tau = multibody_rnea_ext(mb, q, dq, ddq);
  auto end_mb_rnea = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> duration_mb_rnea = end_mb_rnea - start_mb_rnea;

  std::cout << "rigidbody-rs: " << cast_darray(tau).transpose() << std::endl;
  std::cout << "multibody_rnea execution time: " << duration_mb_rnea.count() << " microseconds" << std::endl;
    
  return 0;
}
