#pragma once
#include "define.h"

class Car
{
private:
	int id;
	int location;
	int status;
	int speed;
	int plantime;
	int idCurRoad;//当前车所在的道路ID
	int idCurLane;//当前车所在的车道ID
	int dirCross;//标志在路口的状态 
	int idCrossFrom;//车的出发路口
	int idCrossTo;//车的终止路口
	std::vector<int> path;

public:
	//默认构造函数
	Car() {};
	//重载构造函数，直接将6个参数传入
	Car(int id, int idCrossFrom, int idCrossTo, int speed, int plantime, int status) :
		id(id), idCrossFrom(idCrossFrom), idCrossTo(idCrossTo), speed(speed), plantime(plantime), status(status) {}
	~Car() {};
	//获得car的数据
	int getID() { return id; };
	int getIdForm() { return idCrossFrom; };
	int getIdTo() { return idCrossTo; };
	int getPlaneTime() { return plantime; };
	int getLocation() { return location; };
	int getStatus() { return status; };
	int getSpeed() { return speed; };
	std::vector<int> getPath() { return path; };
	int getIdCurRoad() { return idCurRoad; };
	int getIdCurLane() { return idCurLane; };
	int getDirCross() { return dirCross; };
	int getPathSize() { return path.size(); };
	//修改car的数据
	void putID(int n_id) { id = n_id; };
	void putIdForm(int n_idfrom) { idCrossFrom = n_idfrom; };
	void putIdTo(int n_idto) { idCrossTo = n_idto; };
	void putPlaneTime(int n_plantime) { plantime = n_plantime; };
	void putLocation(int n_location) { location = n_location; };
	void putStatus(int n_status) { status = n_status; };
	void putSpeed(int n_speed) { speed = n_speed; };
	void putPath(std::vector<int> n_path) { path = n_path; };
	void putIdCurRoad(int n_idcurroad) { idCurRoad = n_idcurroad; };
	void putIdCurLane(int n_idcurlane) { idCurLane = n_idcurlane; };
	void putDirCross(int n_dircross) { dirCross = n_dircross;  };
};

class Cross
{
public:
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

	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;
};