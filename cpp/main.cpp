// main.cpp
#include <iostream>
#include <iomanip>
#include "rigidbody.h"

int main() {
  Multibody* mb = multibody_new();

  float q[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float dq[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  float ddq[7] = {5.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  float* tau = multibody_rnea(q, dq, ddq);

  std::cout << "RNEA Result: " << std::endl;
  for (int i = 0; i<7; i++)
    {
      std::cout << std::setprecision(3) << " " << tau[i];
    }
  std::cout << std::endl;


  multibody_free(mb);
  return 0;
}
