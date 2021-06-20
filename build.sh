#! /bin/bash

Help()
{
   # Display Help
   echo "How to build yape-ros folder."
   echo
   echo "Syntax: ./build.sh <build type> <build tests>"
   echo "options:"
   echo "<build type>: RELEASE or DEBUG."
   echo "<build tests>: ON or OFF."
   echo
}

if [ "$#" -ne 2 ]; then
    Help
    exit 1
fi

BUILD_TYPE=$1
BUILD_TESTS=$2

chmod +x install_dependencies.sh
./install_dependencies.sh
mkdir build && cd build && cmake -DCMAKE_BUIL_TYPE=${BUILD_TYPE} -DBUILD_TESTS=${BUILD_TESTS} .. && make
