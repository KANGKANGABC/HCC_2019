#ifndef DATA_CENTER_H_
#define DATA_CENTER_H_

#include "lib_io.h"
#include "define.h"
#include "Road.h"
#include "dijkstra.h"


class DataCenter
{

	friend class Graph_DG;
public:
	DataCenter();
	DataCenter(char *data_road[MAX_ROAD_NUM], int road_count, char *data_car[MAX_CAR_NUM], int car_count, char *data_cross[MAX_CROSS_NUM], int cross_count);
	~DataCenter();


	friend class Graph_DG;
	friend class Scheduler;

	//读入数据
	void readRoadData();
	void readCarData();
	void readCrossData();

	//获取点和边的数量
	int getRoadNum();
	int getCrossNum();

	//获得路长邻接矩阵
	std::vector<std::vector<int> > getArc();
	//获得道路限速邻接矩阵
	std::vector<std::vector<int> > getRoadvArc();

	//将结果写出到result.txt
	void writeResult(const char *filename);

	void writeResultWithTime(const char *filename);

	//获得规划的路径
	void getPath();

	enum
	{// 车辆运行状态 //请参考论坛中关于任务调度的解释
		SLEEPING,	// 等待出发（车库中）
		WAITTING,	// 等待行驶 
		FINESHED,   // 终止车辆 
	};

	//所有道路
	//道路对象的指针
	Road * road;

	//所有路口的指针
	Cross * cross;

	//所有Car的指针
	Car * car;

	//经过排序的car的指针
	vector<Car*> carOrderTime;

	int m_road_num;//ROAD数量
	int m_car_num;//CAR数量
	int m_cross_num;//CROSS数量
	int car_speed_num; //car可能的速度类型

	std::string result;//输出结果存储矩阵
	int vexnum, edge;
	std::vector<std::vector<int> > tmp;

	//根据速度存储车辆
	std::vector<std::vector<Car> > carsBySpeed;

	//动态调度器计算出来的道路情况矩阵
	std::vector<std::vector<float> > graphRoadStatusByDS;

private:

	char **inputRoadData;//输入道路数据
	char **inputCarData;//输入道路数据
	char **inputCrossData;//输入道路数据

	//道路有向图邻接矩阵
	std::vector<std::vector<int> > graphRoad;	//距离邻接矩阵，不邻接的点用正无穷表示
	std::vector<std::vector<int> > graphMaxSpeed;	//道路最大速度邻接矩阵，不邻接的点用0表示
	//std::vector<std::vector<float> > timeGraph;		// 时间邻接矩阵，不邻接的点用正无穷表示

	//存储车辆的速度种类的向量
	std::vector<int> speedType;

	//CrossToRoad转换表
	std::vector<std::vector<int> > graphC2R;

	//路口信息表
	//(id,roadId,roadId,roadId,roadId)
	std::vector<std::vector<int> > crossList;

	//Car调度任务表
	std::vector<std::vector<int> > carTask;

	//路径列表
	std::vector<std::vector<int> > carPathList;

};


#endif