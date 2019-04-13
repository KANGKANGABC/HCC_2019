#pragma once
#include "define.h"

class Car
{
public:
	int index;
	int id;
	int location;
	int status;
	int speed;
	int plantime;//计划驶出时间
	int starttime;//真实驶出时间
	int starttimeAnswer;//写出answer的starttime
	int timeArrived;//记录车辆到达时间
	int idCurRoad;//当前车所在的道路ID
	int idCurLane;//当前车所在的车道ID
	int dirCross;//标志在路口的状态 
	int idCrossFrom;//车的出发路口
	int idCrossTo;//车的终止路口
	int time;
	int dirMap;//车在地图上的方向，分为（++）（--）（+-）（-+）四种
	int priority;//优先级
	int preset;//是否预置path
	bool isChanged;//预置车辆，且path被修改
	std::vector<int> path;
}; 

class Cross
{
public:
	int index;
	int id;
	int roadID_T; //顺时针第1个 Top
	int roadID_R; //顺时针第2个 Right
	int roadID_D; //顺时针第3个 Down
	int roadID_L; //顺时针第4个 Left
	std::vector<int> roadID;//按顺序存储上面四个方向，便于遍历

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

	int index;
	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;
};

