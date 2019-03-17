#!/bin/bash
sh build.sh
cd bin
./CodeCraft-2019 ../config_test/car.txt ../config_test/road.txt ../config_test/cross.txt ../config_test/answer.txt
./CodeCraft-2019 ../train1/car.txt ../train1/road.txt ../train1/cross.txt ../train1/answer.txt
