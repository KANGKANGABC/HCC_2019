#pragma once
#include "define.h"
#include "Road.h"
#include "DataCenter.h"
#include "dijkstra.h"

class DataCenter;

class Scheduler
{
public:
	Scheduler(DataCenter &dc);
	~Scheduler();
	int getSysTime();
	//基于动态调度器规划路径
	void getPathByScheduler();
	//获得路径,为每辆车规划路径
	void getPath();
	//获得路径,为每辆车规划路径，基于时间
	void getPathByTime();
	void getPathByTime_dynamic(); //根据1-100 和101-199车的轨迹，更新第200辆车的邻接矩阵

	int vexnum, edge;
	std::vector<std::vector<int> > tmp;
	std::vector<std::vector<int> > tmp1;
private:
	int timecount;
	int num_CarsScheduling;//正在调度的car数量
	int num_CarsPut;//已经发车的car数量
	int num_Roads;//道路数量
	int num_Crosses;//路口数量
	int num_Cars;//车数量
	int time_Scheduler;//调度器运行时间
	Road *roads;//所有的道路（道路对象数组的指针）
	Cross *crosses;//所有的路口（路口对象数组的指针）
	Car *cars;//所有的车
	//CrossToRoad转换表
	std::vector<std::vector<int> > graphC2R;

	//动态调度器计算出来的道路情况矩阵
	std::vector<std::vector<float> > graphRoadStatusByDS;

	/*所有road上的车辆行进，直到该车辆行驶变成等待状态或者终止状态*/
	void driveAllCarsJustOnRoadToEndState();
	
	/*让该车前进*/
	int driveCar(Car car, int indexCar);//indexCar为该车在车道的位置

	/*将该车加入道路行驶*/
	void addCar(Car car,int i);//i为该车在cars[]中的下标，便于加入车失败时延后时间片

	/*将该车加入道路行驶,动态规划路径*/
	void addCar(Car car, int i, Graph_DG &graph);

	//判断某cross的某road是否可以行驶进入//输入ID为修正前的ID
	int isCanEnter(int idRoad, int idCross);//如果返回值-1，代表不可加入，否则返回可驶入的lane ID
		
	//判断某cross的某road是否存在需要直行的车
	bool isBeDD(int idRoad, int idCross);

	//判断某cross的某road是否存在需要左转的车
	bool isBeLEFT(int idRoad, int idCross);

	//输入当前道路和下一道路，以及路口ID，计算出车辆从当前道路驶入下一道路的方向
	int getCrossDir(int idCurRoad, int idNextRoad, int idNextCross);

	//如果车辆从当前道路驶向下一路口，因为道路限速可能与区别，这里根据官方的规则计算车辆能行驶的最大距离
	//如果能行驶的最大距离为0，那么只能停在路口，等待下一次调度
	int getCrossDistance(Car car, int idCurRoad, int idNextRoad);

	//将该车行驶到下个road 根据假设AA:此时不存在有车到达终点
	void driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location);

	//判断该车能否在某路口转向并行驶
	bool isCanDriveToNextRoad(Car car, int dir, int idCross);//dir为目标行驶方向

	//车库中的车辆上路行驶
	void driverCarInGarage();

	//车库中的车辆上路行驶,动态更新其路径
	void driverCarInGarageDynamic(Graph_DG &graph);

	//打印车辆状态
	void putCarStatus(Car car);

	//输出所有车辆状态
	void putAllCarStatus();

	//输出所有道路状态
	void putAllRoadStatus();

	//获得该cross的对应优先级道路ID，如果道路ID为-1则返回-1
	int getFirstRoadFromCross(int idCross,int index);

	//获得某road在某cross的某方向
	int getDirByRoadCrossDir(int idCross,int idRoad);

	//某个road上的某个channel车辆行进，仅处理该车道上行驶且能到达终止状态的车
	void driveAllCarsJustOnOneChannelToEndState(int idRoad, int idCross, int idChannel);

};

