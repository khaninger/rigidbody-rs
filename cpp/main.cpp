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
  
  auto joint_id = (pinocchio::JointIndex)model.njoints;
  //std::cout << std::setprecision(3) << data.tau.transpose() << std::endl;
  return data.tau;
}

Eigen::VectorXd cast_array(float* arr) {
  Eigen::Map<Eigen::VectorXf> vec_float(arr, 7); 
  Eigen::VectorXd vec = vec_float.cast<double>();
  return vec;
}


void print(Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::VectorXd ddq) {
  std::cout << "  q:" << q.transpose() << std::endl;
  std::cout << " dq:" << dq.transpose() << std::endl;
  std::cout << "ddq:" << ddq.transpose() << std::endl;

}

int main() {
  Multibody* mb = multibody_new();

  float q[7]   = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float dq[7]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float ddq[7] = {5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  print(cast_array(q), cast_array(dq), cast_array(ddq));
  
  Eigen::VectorXd pin_tau = runRNEA(cast_array(q), cast_array(dq), cast_array(ddq));
  std::cout << "pinocchio:    " << pin_tau.transpose() << std::endl;

  float* tau = multibody_rnea(q, dq, ddq); 
  std::cout << "rigidbody-rs: " << cast_array(tau).transpose() << std::endl;
  
  return 0;
}
