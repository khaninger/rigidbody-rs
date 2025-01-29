#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>


/// An explicit revolute joint with a single degree of freedom
struct RevoluteJoint;

struct Multibody {
  RevoluteJoint [7];
};


extern "C" {

void multibody_free(Multibody *mb_ptr);

Multibody *multibody_new();

float (multibody_rnea(Multibody *mb_ptr,
                      const float (*q)[7],
                      const float (*dq)[7],
                      const float (*ddq)[7]))[7];

}  // extern "C"
