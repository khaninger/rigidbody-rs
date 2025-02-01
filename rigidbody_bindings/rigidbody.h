#ifndef MULTIBODY_INTERFACE_H
#define MULTIBODY_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

struct Multibody; // Forward declaration

// Function declarations
Multibody* multibody_new();
float* multibody_rnea(const float q[7], const float dq[7], const float ddq[7]);
void multibody_free(Multibody* mb);

#ifdef __cplusplus
}
#endif

#endif // MULTIBODY_INTERFACE_H
