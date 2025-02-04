#ifndef MULTIBODY_INTERFACE_H
#define MULTIBODY_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

struct Multibody; // Forward declaration

// Function declarations
Multibody* multibody_new();
double* multibody_rnea(const double q[7], const double dq[7], const double ddq[7]);
void multibody_free(Multibody* mb);

#ifdef __cplusplus
}
#endif

#endif // MULTIBODY_INTERFACE_H
