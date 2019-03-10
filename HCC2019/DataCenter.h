#ifndef DATA_CENTER_H_
#define DATA_CENTER_H_

#include "lib_io.h"
#include "define.h"

class DataCenter
{
public:
	DataCenter();
	DataCenter(char * data_road[MAX_ROAD_NUM], int road_count);
	~DataCenter();

	//读入数据
	void readRoadData();
private:

	//分割数据
	void splitRoadData();

	//输入道路数据
	char **inputRoadData;

	int m_road_num;//道路数据行数

	//道路有向图邻接矩阵
	std::vector<std::vector<int>> graphRoad;
		

	

};


#endif