packet loss demo

# 1. Compile the libraries needed for C Node

The following assumes that you have installed dora. If not, please refer to the [documentation](https://dora-rs.ai/zh-CN/docs/guides/Installation/installinghttps://dora-rs.ai/zh-CN/docs/guides/Installation/installing).

~~~bash
cd ~ && mkdir dora_project && cd dora_project
git clone https://github.com/dora-rs/dora.git
cd ~/dora_project/dora/examples/c++-dataflow
cargo run --example cxx-dataflow  # compile C++ node
cargo build -p dora-node-api-c --release  # compile dora-node-api-c 
~~~

You should ensure that your dora repo and autoware.universe repo is in the `~/dora_project`  like this:

~~~plaintext
~/
|-- dora_project
	|-- dora/
	|-- autoware.universe/
~~~

 

# 2. Confirm clang path

and then,Please check if your clang is installed via apt. If you installed clang via apt, you don't need to do anything in this step. If not, please run `which clang++` and then change `set(CMAKE_CXX_COMPILER "/usr/bin/clang++")` to your own path in CMakeLists.txt. For example, if you clang path like this:

~~~bash
❯ which clang
/home/xiling2/Desktop/tools/clang+llvm-12.0.0-x86_64-linux-gnu-ubuntu-20.04/bin/clang
~~~

you should modify CMakeLists.txt:

~~~vim
# set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_CXX_COMPILER "/home/xiling2/Desktop/tools/clang+llvm-12.0.0-x86_64-linux-gnu-ubuntu-20.04/bin/clang++")
~~~





# 3.Preparing for the test

~~~bash
sudo apt-get install libyaml-cpp-dev
sudo apt-get install nlohmann-json3-dev
sudo apt-get install libmsgpack-dev
sudo apt-get install pkg-config



cd ~/dora_project
git clone https://github.com/dora-rs/autoware.universe.git
cd autoware.universe
git checkout feature/autoware_dora

cd ~/dora_project/autoware.universe/tools/C_Python_test
mkdir build && cd build
cmake ..
make
~~~



# 4.start testing

~~~bash
cd ~/dora_project/autoware.universe/tools/C_Python_test
dora up
dora start dataflow.yml  --name test
dora logs test node_B
~~~

if everthing is ok you should see the expected result and what actually received from this logs display. We found more nodes involved the most chances the package lost can be found. From this example you may not see data lost each time when you run as this test tool only simulated two nodes. But keep trying and you will be able to reproduce it.
