#pragma once
#include "define.h"

class Lane
{
public:
	bool dir;//��������
	std::vector<int> laneCarId;//��¼ÿ�������ϵĳ���ID
};
class Road
{
public:
	Road();
	~Road();

	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;
};

