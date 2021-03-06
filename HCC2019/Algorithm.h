#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "define.h"
#include "DataCenter.h"
#include "dijkstra.h"
#include "Scheduler.h"

class Algorithm
{
//Vals
public:

private:
	DataCenter * m_dc;

	int num_Cars;
	int num_Roads;
	int num_Cross;
	int num_path_preset;
	Car *cars;//所有的车（车对象数组的指针）
	Road *roads;//所有的道路（道路对象数组的指针）
	Cross *crosses;//所有的路口（路口对象数组的指针）

	//CrossToRoad转换表
	std::vector<std::vector<int> > graphC2R;
	//存储车辆的速度种类的向量
	std::vector<int> speedType;
	//按照出发时间将车辆重排序

	vector<Car> reorderCar;//按照出发时间将车辆重排序
	vector<Car> qCar;	//按照速度将车辆重排序

//Functions
public:
	Algorithm(DataCenter &dc);
	~Algorithm();
	//路径：最短时间路径 时间：根据速度分批出发 调参：调度器自动调参
	void ShortestTime_SpeedBasic_AutoPara();

	//路径：最短时间路径 + 静态路径优化 时间：根据速度分批出发 调参：调度器自动调参
	void StaticAnalysis_SpeedBasic_AutoPara();

	//路径：最短时间路径 + 调度器动态优化 时间：根据速度分批出发 调参：调度器自动调参
	void DynamicPathByScheduler_SpeedBasic_AutoPara(int w);

	//路径：最短时间路径 + 静态路径优化（归一化） 时间：根据速度分批出发+同时刻同地点只发一辆 调参：调度器自动调参
	void StaticAnalysisNor_SpeedBasicNoSame_AutoPara(int para);

	//路径：最短时间路径  时间：根据速度分批出发+根据路况挑选合适的车 调参：调度器自动调参
	void ShortestTime_SpeedBasicRoadStatus_AutoPara(int para);

	//修改死锁
	void unlockDead(int para);

	//尝试解锁
	void tryUnlockDead(Scheduler sd, int para);

private:
	/*获得路径的方法*/
	void getPath();//基于最短路径算法获得路径
	void getPath_StaticAnalysis();//最短路径算法+静态分析
	void getPath_StaticAnalysisNor();

	/*获得出发时间的方法*/
	void getStartTime_BySpeed(int para);
	void ReOrderStartBySpeedAndStartCross(int para);//根据速度和出发点重新安排出发时间

	/*辅助函数*/
	int getPartition(vector<Car> &reorderCar, int begin, int end);
	void quicksort(vector<Car> &reorderCar, int begin, int end);
	void swap(int i, int j);
	void quicksort(int begin, int end);
	void reorderCars(vector<Car> &reorderCar);
	void reorderCarsStarttime();

	//通过id查找index
	map<int, int> mapId2IndexCar;
	map<int, int> mapId2IndexRoad;
	map<int, int> mapId2IndexCross;

	int id2indexCar(int id);
	int id2indexRoad(int id);
	int id2indexCross(int id);

};

#endif
