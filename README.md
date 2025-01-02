This is a lightweight, URDF-compatible kinematics and dynamics library in Rust designed for robotics.

## Do I need this?
The goals of this library are to support the following for rigid-body manipulators:
1. Support realtime control
   - Feedforward dynamic compensation
   - Task-space control
2. Dynamic simulation 
3. Model-predictive control 
3. Explore suitability and performance of Rust features for robotics development (in particular: memory safety, dev tools, LLVM-level autodiff)

### Features
- [x] Forward/inverse kinematics
- [x] Forward/inverse dynamics
- [x] Parse from URDF
- [ ] Differentiability
- [ ] Contact dynamics
- [ ] Visualization
- [ ] Non-rigid body dynamics

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
