// main.cpp
#include <iostream>
#include "rigidbody.h"

int main() {
    Multibody* mb = multibody_new();

    float q[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float dq[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float ddq[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float* tau = multibody_rnea(q, dq, ddq);
    std::cout << "RNEA Result: " << tau[0] << tau[1] << tau[2] << tau[3] << tau[4] << tau[5] << tau[6] << std::endl;

    multibody_free(mb);
    return 0;
}
