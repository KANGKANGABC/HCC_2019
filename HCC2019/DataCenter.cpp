#include "DataCenter.h"
#include "Tools.h"
#include <iostream>
#include <algorithm>

DataCenter::DataCenter()
{
}

DataCenter::DataCenter(char *data_road[MAX_ROAD_NUM],int road_count)
{
	inputRoadData = data_road;
	m_road_num = road_count;
	//将邻接矩阵大小设置为36
	graphRoad.resize(36);
	for (int i = 0; i < 36; ++i) {
		graphRoad[i].resize(36);
	}

}
DataCenter::~DataCenter()
{
}
void DataCenter::readRoadData()
{
	printf("readRoadData\n");
	for (int i = 1; i < m_road_num; ++i)//忽略第0行数据
	{
		std::string roadInfo = inputRoadData[i];
		std::vector<std::string> sp = Tools::split(roadInfo, ", ");
		if (sp[6].substr(0, 1) == "1")
		{
			graphRoad[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[1]);
			graphRoad[std::stoi(sp[5]) - 1][std::stoi(sp[4]) - 1] = std::stoi(sp[1]);

		}
		else
		{
			graphRoad[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[1]);
		}

	}
	printf("readRoadData done!\n");
}

void DataCenter::splitRoadData()
{
}
