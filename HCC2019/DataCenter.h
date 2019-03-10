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

	//��������
	void readRoadData();
private:

	//�ָ�����
	void splitRoadData();

	//�����·����
	char **inputRoadData;

	int m_road_num;//��·��������

	//��·����ͼ�ڽӾ���
	std::vector<std::vector<int>> graphRoad;
		

	

};


#endif