#pragma once
#include "define.h"
class Cross
{
public:
	int id;
	std::vector<Car> crossCar;//记录目前在路口等待调度的车辆信息
	int roadID_T; //顺时针第1个 Top
	int roadID_R; //顺时针第2个 Right
	int roadID_D; //顺时针第3个 Down
	int roadID_L; //顺时针第4个 Left

};
class Car
{
public:
	int id;
	int location;
	int status;
	int speed;
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

	enum
	{// 车道方向
		FORWARD,	// 正向
		BACKWARD,	// 逆向
	};
};

