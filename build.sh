# How to build and run the main.cpp linking against the rigidbody-rs lib
cargo build
g++ -o main src/main.cpp -Ltarget/debug -l rigidbody
LD_LIBRARY_PATH=target/debug ./main
