#ifndef DATA_CENTER_H_
#define DATA_CENTER_H_

#include "lib_io.h"
#include "define.h"

class DataCenter
{
public:
	DataCenter();
	DataCenter(char *data_road[MAX_ROAD_NUM], int road_count, char *data_car[MAX_CAR_NUM], int car_count, char *data_cross[MAX_CROSS_NUM], int cross_count);
	~DataCenter();

	//读入数据
	void readRoadData();
	void readCarData();
	void readCrossData();
private:

	//分割数据
	void splitRoadData();

	char **inputRoadData;//输入道路数据
	char **inputCarData;//输入道路数据
	char **inputCrossData;//输入道路数据

	int m_road_num;//ROAD数据行数
	int m_car_num;//CAR数据行数
	int m_cross_num;//CROSS数据行数

	//道路有向图邻接矩阵
	std::vector<std::vector<int>> graphRoad;

	//Car调度任务向量
	std::vector<std::vector<int>> carTask;
	

	

};


#endif