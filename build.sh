# How to build and run the main.cpp linking against the rigidbody-rs lib
cargo build --example rigidbody
g++ -o main cpp/main.cpp -Ltarget/debug/examples -lrigidbody
LD_LIBRARY_PATH=target/debug/examples ./main
