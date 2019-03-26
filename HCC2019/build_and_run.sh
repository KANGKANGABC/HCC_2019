#!/bin/bash
sh build.sh
cd bin
./CodeCraft-2019 ../train1/car.txt ../train1/road.txt ../train1/cross.txt ../train1/answer.txt > train1.log
./CodeCraft-2019 ../train2/car.txt ../train2/road.txt ../train2/cross.txt ../train2/answer.txt > train2.log
