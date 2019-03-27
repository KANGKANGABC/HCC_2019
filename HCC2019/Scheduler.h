#pragma once
#include "define.h"
#include "Road.h"
#include "DataCenter.h"
#include "dijkstra.h"
#include "map"

class DataCenter;

class Scheduler
{
public:
	Scheduler(DataCenter &dc);
	~Scheduler();
	int getParaByScheduler();//请将初始参数设置为70以上
	int getSysTime();
	int getSysTimeChangePath(int para);
	int getSysTimeChangeTime(int para);//边跑调度器边修改出发时间
	//基于动态调度器规划路径
	int getPathByScheduler(int para);
	//获得路径,为每辆车规划路径
	//尝试解决死锁
	int unlockDead(int para);
	int getTimeByNoSameStartCross(int para);
	void getPath();
	//获得路径,所有权重为1
	void getPathWeightOne();
	//获得路径后，规划出发时间,para为参数
	void getStartTime(int para);
	void getStartTime_loadbalance(int carnum);
	//获得路径,为每辆车规划路径，基于时间
	void getPathByTime();
	void getPathByTime_reorderCars();
	void reorderCars();
	//车辆按出发时间重排序后进行静态规划
	void getPathByTime_dynamic(); //根据1-100 和101-199车的轨迹，更新第200辆车的邻接矩阵
	void swap(int i, int j);
	void quicksort(int begin, int end);
	void getTimeByDir(int para);//根据车的行驶方向发车（++）和（--）的一起跑 （+-）和（-+）的一起跑 
	void ReOrderStartByTime(int para);//根据行驶时间重新安排出发时间
	void ReOrderStartBySpeed(int para);//根据行驶时间重新安排出发时间
	void ReOrderStartBySpeedAndStartCross(int para);//根据速度和出发点重新安排出发时间
	bool addCarandChangeSTime(Car car);//往道路中添加车辆，并且通过改变出发时间留出空位
	int vexnum, edge;
	std::vector<std::vector<int> > tmp;
	std::vector<std::vector<int> > tmp1;

	std::map<string, float > mapForJamDegree;	//存储道路的拥挤程度
	//对拥挤度map的两个操作
	bool judgement(map<string, float > &mapForJamDegree, vector<int> path);
	void mapUpdate(map<string, float > &mapForJamDegree, int RoadId, float percent);

	int num_changeSTime;

	//反映调度器状态的数据结构
	std::vector<int> vec_numCarsInRoadPerTime;//每个时间片车的数量
private:
	int num_CarsScheduling;//正在调度的car数量
	int num_CarsPut;//已经发车的car数量
	int num_Roads;//道路数量
	int num_Crosses;//路口数量
	int num_Cars;//车数量
	int time_Scheduler;//调度器运行时间
	Road *roads;//所有的道路（道路对象数组的指针）
	Cross *crosses;//所有的路口（路口对象数组的指针）
	Car *cars;//所有的车

	//通过id查找index
	map<int, int> mapId2IndexCar;
	map<int, int> mapId2IndexRoad;
	map<int, int> mapId2IndexCross;
  
	vector<Car> qCar;	//按照出发时间将车辆重排序

	//存储车辆的速度种类的向量
	std::vector<int> speedType;

	std::deque<Car> carsWaitInGarage;//上一时间片未驶出，等待驶出的车
	std::deque<Car> carsInGarage;//此时间片待出发的车
	std::deque<Car> carsDeadLock;//死锁的车

	//CrossToRoad转换表
	std::vector<std::vector<int> > graphC2R;

	//动态调度器计算出来的道路情况矩阵
	std::vector<std::vector<float> > graphRoadStatusByDS;

	/*所有road上的车辆行进，直到该车辆行驶变成等待状态或者终止状态*/
	void driveAllCarsJustOnRoadToEndState();
	

	int driveCarNew(Car car);

	Car getCarFromRoad(int idRoad,int dir);

	//第一阶段车行驶
	void driveCarStep1(Car car, int indexCar);

	/*将该车加入道路行驶*/
	bool addCar(Car car,int i);//i为该车在cars[]中的下标，便于加入车失败时延后时间片

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
	void driverCarInGarageDynamic(Graph_DG &graph,int para);

	//车库中的车辆上路行驶,动态更新其出发时间
	void driverCarInGarageChangeTime(Graph_DG &graph, int para);

	//打印车辆状态
	void putCarStatus(Car car);

	//输出所有车辆状态
	bool putAllCarStatus();

	//输出所有道路状态
	void putAllRoadStatus();

	//获得该cross的对应优先级道路ID，如果道路ID为-1则返回-1
	int getFirstRoadFromCross(int idCross,int index);

	//获得某road在某cross的某方向
	int getDirByRoadCrossDir(int idCross,int idRoad);

	//某个road上的某个channel车辆行进，仅处理该车道上行驶且能到达终止状态的车
	void driveAllCarsJustOnOneChannelToEndState(int idRoad, int idCross, int idChannel);

	//根据时间周期安排出发时间
	void getPlantimeByPeriod(int period);

	int id2indexCar(int id);
	int id2indexRoad(int id);
	int id2indexCross(int id);
	
	//调度器初始化
	int SchedulerInit();
	//调度器主要逻辑代码，便于多算法复用
	int SchedulerCore();

};

