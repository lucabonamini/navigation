#! /bin/bash
sudo apt install -y libeigen3-dev libgoogle-glog-dev libgflags-dev
cd control/mpc/
chmod +x install_ipopt.sh setup_dependencies.sh
./setup_dependencies.sh