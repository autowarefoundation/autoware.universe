## get llvm
yes Y | sudo apt-get install llvm
yes Y | sudo apt-get install pthread
yes Y | sudo apt install libomp-dev

## get clang
# todo

## install CppAD
mkdir /tmp/cppadcg_deps
cd /tmp/cppadcg_deps

#wget https://github.com/coin-or/CppAD/archive/20210000..tar.gz
#tar -xzf 20200000.3.tar.gz
#cd CppAD-20200000.3

#wget https://github.com/coin-or/CppAD/archive/20210000.7.tar.gz
#tar -xzf 20210000.7.tar.gz
#cd CppAD-20210000.7

wget https://github.com/coin-or/CppAD/archive/refs/tags/20220000.4.tar.gz
tar -xzf 20220000.4.tar.gz
cd CppAD-20220000.4

mkdir build
cd build
cmake -Dcppad_prefix:PATH='/usr/local' ..
sudo make install

## install CppADCodeGen
git clone https://github.com/joaoleal/CppADCodeGen.git /tmp/CppADCodeGen
cd /tmp/CppADCodeGen
git checkout v2.4.3
mkdir -p build
cd build
cmake ..
make
sudo make install
