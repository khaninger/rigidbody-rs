// main.cpp
#include <iostream>
#include <iomanip>
#include "rigidbody.h"

#include <Eigen/Dense>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"


Eigen::VectorXd runRNEA(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& dq,
                        const Eigen::VectorXd& ddq) {
  pinocchio::Model model;
  pinocchio::urdf::buildModel("assets/fr3.urdf", model);
  
  pinocchio::Data data(model);

  // Benchmarking
  auto start = std::chrono::high_resolution_clock::now();
  pinocchio::rnea(model, data, q, dq, ddq);
  auto end = std::chrono::high_resolution_clock::now();
  
  // Calculate elapsed time
  //std::chrono::duration<double, std::milli> elapsed = end - start;
  //std::cout << std::setprecision(4) << "Joint configuration: " << q.transpose() << " - Time taken for forward kinematics: " << elapsed.count() << " ms" << std::endl;
  
  return data.tau;
}

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

  double q[7]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  double dq[7]  = {0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
  double ddq[7] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f};

  Eigen::VectorXd q_ = cast_darray(q);
  Eigen::VectorXd dq_ = cast_darray(dq);
  Eigen::VectorXd ddq_ = cast_darray(ddq);
    
  std::cout << "  q:" << q_.transpose() << std::endl;
  std::cout << " dq:" << dq_.transpose() << std::endl;
  std::cout << "ddq:" << ddq_.transpose() << std::endl;
  
  Eigen::VectorXd pin_tau = runRNEA(q_, dq_, ddq_);
  std::cout << "pinocchio:    " << pin_tau.transpose() << std::endl;

  double* tau = multibody_rnea(q, dq, ddq); 
  std::cout << "rigidbody-rs: " << cast_darray(tau).transpose() << std::endl;
  
  return 0;
}
