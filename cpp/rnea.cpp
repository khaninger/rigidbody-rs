//rnea.cpp
#include <iostream>
#include <chrono>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

void runRNEA(const pinocchio::Model& model,
             const Eigen::VectorXd& q,
             const Eigen::VectorXd& dq,
             const Eigen::VectorXd& ddq) {
    pinocchio::Data data(model);

    // Benchmarking
    auto start = std::chrono::high_resolution_clock::now();
    pinocchio::rnea(model, data, q, dq, ddq);
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate elapsed time
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << std::setprecision(4) << "Joint configuration: " << q.transpose() << " - Time taken for forward kinematics: " << elapsed.count() << " ms" << std::endl;

    auto joint_id = (pinocchio::JointIndex)model.njoints;
    std::cout << std::setprecision(3) << data.tau.transpose() << std::endl; 
}

int main() {
    pinocchio::Model model;
    pinocchio::urdf::buildModel("assets/fr3.urdf", model);

    // Define joint configurations to test
    Eigen::VectorXd q1 = Eigen::VectorXd::Ones(model.nq); // All 1.0
    Eigen::VectorXd q2 = Eigen::VectorXd::Zero(model.nq); // All 0.0
    Eigen::VectorXd q3 = Eigen::VectorXd::Constant(model.nq, -1.0); // All -1.0

    // Define joint velocities and accelerations (example values)
    Eigen::VectorXd dq = Eigen::VectorXd::Zero(model.nv); // Zero velocities
    Eigen::VectorXd ddq = Eigen::VectorXd::Zero(model.nv); // Zero accelerations
    ddq[0] = 5.;

    std::cout << "ddq" << ddq.transpose() << std::endl;
  
    // Run RNEA for each configuration
    runRNEA(model, q1, dq, ddq);
    runRNEA(model, q2, dq, ddq);
    runRNEA(model, q3, dq, ddq);
  
    return 0;
}
