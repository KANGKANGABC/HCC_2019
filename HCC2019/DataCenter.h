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

	//��������
	void readRoadData();
	void readCarData();
	void readCrossData();
private:

	//�ָ�����
	void splitRoadData();

	char **inputRoadData;//�����·����
	char **inputCarData;//�����·����
	char **inputCrossData;//�����·����

	int m_road_num;//ROAD��������
	int m_car_num;//CAR��������
	int m_cross_num;//CROSS��������

	//��·����ͼ�ڽӾ���
	std::vector<std::vector<int>> graphRoad;

	//Car������������
	std::vector<std::vector<int>> carTask;
	

	

};


#endif