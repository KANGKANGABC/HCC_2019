#pragma once
#include "define.h"
#include "Road.h"
#include "DataCenter.h"

class DataCenter;

class Scheduler
{
public:
	Scheduler(DataCenter &dc);
	~Scheduler();
	int getSysTime();
	//获得路径,为每辆车规划路径
	void getPath();
private:
	int num_CarsScheduling;//正在调度的car数量
	int num_Roads;//道路数量
	int num_Crosses;//路口数量
	int num_Cars;//车数量
	int time_Scheduler;//调度器运行时间
	Road *roads;//所有的道路（道路对象数组的指针）
	Cross *crosses;//所有的路口（路口对象数组的指针）
	Car *cars;//所有的车

	/*所有road上的车辆行进，直到该车辆行驶变成等待状态或者终止状态*/
	void driveAllCarsJustOnRoadToEndState();
	
	/*让该车前进*/
	void driveCar(Car &car, int indexCar);//indexCar为该车在车道的位置

	/*将该车加入道路行驶*/
	void addCar(Car &car);

	//判断某cross的某road是否可以行驶进入
	int isCanEnter(int idRoad, int idCross);//如果返回值-1，代表不可加入，否则返回可驶入的lane ID
		
	//判断某cross的某road是否存在需要直行的车
	bool isBeDD(int idRoad, int idCross);

	//判断某cross的某road是否存在需要左转的车
	bool isBeLEFT(int idRoad, int idCross);

	//输入当前道路和下一道路，以及路口ID，计算出车辆从当前道路驶入下一道路的方向
	int getCrossDir(int idCurRoad, int idNextRoad, int idNextCross);

	//如果车辆从当前道路驶向下一路口，因为道路限速可能与区别，这里根据官方的规则计算车辆能行驶的最大距离
	//如果能行驶的最大距离为0，那么只能停在路口，等待下一次调度
	int getCrossDistance(Car &car, int idCurRoad, int idNextRoad);

	//将该车行驶到下个road 根据假设AA:此时不存在有车到达终点
	void driverToNextRoad(Car &car, int idNextRoad, int idNextLane, int location);

	//判断该车能否在某路口转向并行驶
	bool isCanDriveToNextRoad(Car &car, int dir, int idCross);//dir为目标行驶方向

	//某个road上的车辆行进，直到该车辆行驶变成等待状态或者终止状态
	void driveAllCarsJustOnOneRoadToEndState(int idRoad, int idCross);

	//车库中的车辆上路行驶
	void driverCarInGarage();

	//打印车辆状态
	void putCarStatus(Car car);

};

