This is a lightweight, URDF-compatible, Featherstone-based kinematics and dynamics library in Rust designed for robotics.

## Do I need this?
The goals of this library are to support the following for rigid-body manipulators:
1. Support realtime control
   - Feedforward dynamic compensation
   - Task-space control
2. Dynamic simulation 
3. Model-predictive control 
3. Explore suitability and performance of Rust features for robotics development (in particular: memory safety, dev tools, LLVM-level autodiff)

It's written to be modular and easier to write your own dynamics-based code in. It's consistent with Featherstone's notation (e.g. spatial velocities / forces are first-order objects).

### Features
- [x] Forward/inverse kinematics
- [x] Forward/inverse dynamics
- [x] Parse from URDF
- [ ] Differentiability
- [ ] Contact dynamics, implicit constraints
- [ ] Visualization
- [ ] Floating base

No support for GPU execution / parallelization planned. 

### Why this instead of {Pinocchio, MuJoCo, Brax, ...}
This has fewer features and is not battle-tested, but is in Rust.


### Why Rust for robotics? 
There's better 'why rust' essays out there, but for specifically for robotics: 
- Developer tooling supports reproduceable / transferrable code: 
  - `cargo` has integrated dependency management which locks the exact version of all libraries in your dependency graph
  - cross-platform support is good, can target most platforms used in robotics
- Language features:
  - `Enum`s in Rust provides nice features for discrete states
    - `match` statements must be exhaustive, i.e. compiler checks that all states handled
    - each `Enum` variant can have different associated data, i.e. you can have `Running(u8, u8)` to store two `int`s that aren't really defined in `Stopped` or other states. 
  - Error handling is excellent, enforced by compiler
  - Memory safety is always good

My opinion is that robotics has limited code re-use because of hardware diveristy and high difficulty from hardware/software complexity (large # of interdependent modules, leaky abstractions, etc). 
ROS2 aims to improve code re-use through packaging dep management, build env, and middleware together. But ROS2 is multilingual (C++ and Python). By just using a language which can meet the performance requirements (i.e. not Python) and which has sensible dependency management (i.e. not C++ or Python), you just need interprocess communication or middleware, a much easier problem.

### Why not Rapier (Rust physics engine)
Rapier has a great physics engine, but has high complexity to support arbitrary changes to the dynamics scene which we don't prioritize here.

Additionally, the forward dynamics essentially closed, and inverse dynamics not exposed over an interface. 


## Getting started
### Build
    - If you just need the rust library `cargo` has you covered.
    - If you want the CPP bindings:
      - a `nix run` will build and run `rigidbody_bindings/main.cpp`
      - the header `rigidbody_bindings/rigidbody.h` is made from `cbindgen`
      - the crate is nix-ified with `cargo2nix`

## Library short intro

### Kinematics
We use Rust's lifetime system to allow us to check the validity of transformations at compile time.

That is, a relative pose contains a reference to its parent coordinate system. If the parent coordinate system is out of scope, this is (conservatively) estimate from the lifetime.

