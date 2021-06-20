#! /bin/bash
sudo apt install -y libeigen3-dev libgoogle-glog-dev libgflags-dev
cd control/mpc/
chmod +x install_ipopt.sh setup_dependencies.sh
./setup_dependencies.sh
cd ../../
git clone https://github.com/lava/matplotlib-cpp.git
cd matplotlib-cpp && mkdir build && cd build && cmake .. && make && sudo make install
cd ../../ && rm -rf matplotlib-cpp
