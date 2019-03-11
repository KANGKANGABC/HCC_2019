#pragma once
#include "define.h"

class Lane
{
public:
	bool dir;//车道方向
	std::vector<int> laneCarId;//记录每个车道上的车辆ID
};
class Road
{
public:
	Road();
	~Road();

	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;
};

