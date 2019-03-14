#ifndef DATA_CENTER_H_
#define DATA_CENTER_H_

#include "lib_io.h"
#include "define.h"
#include "Road.h"

class DataCenter
{
public:
	DataCenter();
	DataCenter(char *data_road[MAX_ROAD_NUM], int road_count, char *data_car[MAX_CAR_NUM], int car_count, char *data_cross[MAX_CROSS_NUM], int cross_count);
	~DataCenter();

	friend class Graph_DG;
	friend class Scheduler;

	//将邻接矩阵写出到文件
	void write_graph();

	//读入数据
	void readRoadData();
	void readCarData();
	void readCrossData();

	//所有道路
	//道路对象的指针
	Road * road;

	//所有路口的指针
	Cross * cross;

	//所有Car的指针
	Car * car;

	int m_road_num;//ROAD数量
	int m_car_num;//CAR数量
	int m_cross_num;//CROSS数量

private:

	char **inputRoadData;//输入道路数据
	char **inputCarData;//输入道路数据
	char **inputCrossData;//输入道路数据

	//道路有向图邻接矩阵
	std::vector<std::vector<int> > graphRoad;

	//路口信息表
	//(id,roadId,roadId,roadId,roadId)
	std::vector<std::vector<int> > crossList;

	//Car调度任务表
	std::vector<std::vector<int> > carTask;

	//路径列表
	std::vector<std::vector<int> > carPathList;
};


#endif