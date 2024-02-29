# # making sure gcc 8 is installed
# sudo apt update && apt install -y gcc-8 g++-8

# # update alternatives
# update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
# update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8

sudo WITH_CUDA=ON python3 setup.py install