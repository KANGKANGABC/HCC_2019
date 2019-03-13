#pragma once
#include "define.h"

class Car
{
public:
	int id;
	int location;
	int status;
	int speed;
	int dirCross;//标志在路口的状态 
	std::vector<int> path;
}; 

class Cross
{
public:
	int id;
	int roadID_T; //顺时针第1个 Top
	int roadID_R; //顺时针第2个 Right
	int roadID_D; //顺时针第3个 Down
	int roadID_L; //顺时针第4个 Left

};

class Lane
{
public:
	bool dir;//车道方向
	int idLane;//车道ID
	std::vector<Car> laneCar;//记录每个车道上的车辆信息

};
class Road
{
public:
	Road();
	~Road();

	//创建车道信息
	void CreateLane();

	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;
};

