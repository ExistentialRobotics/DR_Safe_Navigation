#!/bin/bash
#

# Go to the package root directory
CURRPATH=`pwd`
SCRIPTPATH="$( cd "$(dirname "$0")" ; pwd -P )"
ROOTPATH="$SCRIPTPATH/.."
cd $ROOTPATH

# Check if test binary is complied
if [ -f "$ROOTPATH/bin/test_bresenham" ]; then
  BIN_FOUND=true
else
  BIN_FOUND=false
fi

# Define clean_up function to be used in case we need to compile
clean_up()
{
  echo "CLEANING UP"
  cd $ROOTPATH
  rm -rf build bin lib
  echo "DONE"
}

# Complile the test binary if necessary
if [ "$BIN_FOUND" = false ]; then
  clean_up
  mkdir build && cd build && cmake -DUSE_ROS=OFF .. && make && cd ..
fi

# Run the test script
echo "STARTING TEST"
./bin/test_bresenham
./bin/test_inflate_map
echo "END TEST"

# Cleanup if necessary
if [ "$BIN_FOUND" = false ]; then
  clean_up
fi

# Return to the original path
cd $CURRPATH


