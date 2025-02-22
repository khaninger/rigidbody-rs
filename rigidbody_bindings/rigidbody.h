#ifndef MULTIBODY_INTERFACE_H
#define MULTIBODY_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

struct Multibody; // Forward declaration

// Function declarations
Multibody* multibody_new();
double* multibody_fwd_kin(const Multibody*, const double q[7]);
double* multibody_jac(const Multibody*, const double q[7]);
double* multibody_rnea(const Multibody*, const double q[7], const double dq[7], const double ddq[7]);
double* multibody_crba(const Multibody*, const double q[7]);
void multibody_free(Multibody* mb);

#ifdef __cplusplus
}
#endif

#endif // MULTIBODY_INTERFACE_H
