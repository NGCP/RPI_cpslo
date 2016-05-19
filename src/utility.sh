#!/bin/bash

cd /home/pi/NGCP/RPI_cpslo

git fetch
git pull

mkdir build
cd build
cmake ..
make

./rpi_cpslo

#pid = $(ps -aux | grep rpi_cpslo | tee > (head -n 1) | awk -F ' ' {print $2})
#kill -i $(pid)
