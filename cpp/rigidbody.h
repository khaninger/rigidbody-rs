#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>


struct Multibody;


extern "C" {

void multibody_free(Multibody *mb_ptr);

Multibody *multibody_new();

float* multibody_rnea(Multibody *mb_ptr,
                      float q[7],
                      float dq[7],
                      float ddq[7]);

}  // extern "C"
