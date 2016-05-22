#!/bin/bash

cd /home/pi/NGCP/RPI_cpslo

git fetch
git pull

mkdir build
cd build
cmake ..
make

sudo ./rpi_cpslo ../params.txt &

# Kill rpi_cpslo commands:
#pid = $(ps -aux | grep rpi_cpslo | tee > (head -n 1) | awk -F ' ' {print $2})
#kill -i $(pid)
