#include "DataCenter.h"
#include "Tools.h"
#include <iostream>
#include <algorithm>

DataCenter::DataCenter()
{
}

DataCenter::DataCenter(char *data_road[MAX_ROAD_NUM],int road_count, char *data_car[MAX_CAR_NUM], int car_count, char *data_cross[MAX_CROSS_NUM], int cross_count)
{
	inputRoadData = data_road;
	m_road_num = road_count - 1;//���Ե�һ��ע��
	inputCarData = data_car;
	m_car_num = car_count - 1;//���Ե�һ��ע��
	inputCrossData = data_cross;
	m_cross_num = cross_count - 1;//���Ե�һ��ע��

	//���ڽӾ����С����Ϊ36
	graphRoad.resize(36);
	for (int i = 0; i < 36; ++i) {
		graphRoad[i].resize(36);
	}

	//Car��������������С����
	//Car��������Ϊ������Ҫ���ȵ�Car��
	carTask.resize(m_car_num);
	// |   0     1    2     3       4        5           6         7   |
	// | ����ID ��� �յ� �����ٶ� ����ʱ�� ��ǰ��·ID ��ǰ��·���� ��ǰ״̬|
	for (int i = 0; i < m_car_num; ++i) {
		carTask[i].resize(8);
	}


}
DataCenter::~DataCenter()
{
}
void DataCenter::readRoadData()
{
	printf("readRoadData\n");
	for (int i = 1; i <= m_road_num; ++i)//���Ե�0������
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

void DataCenter::readCarData()
{
	printf("readCarData\n");
	// |   0     1    2     3       4        5           6         7   |
    // | ����ID ��� �յ� �����ٶ� ����ʱ�� ��ǰ��·ID ��ǰ��·���� ��ǰ״̬|
	for (int i = 1; i <= m_car_num; ++i)//���Ե�0������
	{
		std::string carInfo = inputCarData[i];
		std::vector<std::string> sp = Tools::split(carInfo, ", ");
		carTask[i - 1][0] = std::stoi(sp[0].substr(1));//ȥ��������
		carTask[i - 1][1] = std::stoi(sp[1]);
		carTask[i - 1][2] = std::stoi(sp[2]);
		carTask[i - 1][3] = std::stoi(sp[3]);
		carTask[i - 1][4] = std::stoi(sp[4].substr(0, sp[4].size()-1));//ȥ��������
		carTask[i - 1][5] = 0;
		carTask[i - 1][6] = 0;
		carTask[i - 1][7] = 0;
	}
	printf("readCarData done!\n");
}

void DataCenter::readCrossData()
{
}

void DataCenter::splitRoadData()
{
}
