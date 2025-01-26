#include <iostream>
#include <chrono>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

void runForwardKinematics(const pinocchio::Model& model, const Eigen::VectorXd& q) {
    pinocchio::Data data(model);

    // Benchmarking
    auto start = std::chrono::high_resolution_clock::now();
    pinocchio::forwardKinematics(model, data, q);
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate elapsed time
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << std::setprecision(4) << "Joint configuration: " << q.transpose() << " - Time taken for forward kinematics: " << elapsed.count() << " ms" << std::endl;

    auto joint_id = (pinocchio::JointIndex)model.njoints;
    std::cout << std::setw(24) << std::left << model.names[joint_id-1] << ": " << std::fixed
                << std::setprecision(3) << data.oMi[joint_id-1].translation().transpose() << std::endl; 
}

int main() {
    pinocchio::Model model;
    pinocchio::urdf::buildModel("../assets/fr3.urdf", model);

    // Define joint configurations to test
    Eigen::VectorXd q1 = Eigen::VectorXd::Ones(model.nq); // All 1.0
    Eigen::VectorXd q2 = Eigen::VectorXd::Zero(model.nq); // All 0.0
    Eigen::VectorXd q3 = Eigen::VectorXd::Constant(model.nq, -1.0); // All -1.0

    // Run forward kinematics for each configuration
    runForwardKinematics(model, q1);
    runForwardKinematics(model, q2);
    runForwardKinematics(model, q3);
  
    return 0;
}
