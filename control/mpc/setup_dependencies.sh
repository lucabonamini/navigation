#!/bin/bash

# Ipopt
sudo apt install -y gfortran unzip
wget https://www.coin-or.org/download/source/Ipopt/Ipopt-3.12.7.zip && unzip Ipopt-3.12.7.zip && rm Ipopt-3.12.7.zip
cd Ipopt-3.12.7 && ../install_ipopt.sh Ipopt-3.12.7
# CppAD
sudo apt install -y cppad
