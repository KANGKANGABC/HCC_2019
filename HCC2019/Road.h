#pragma once
#include "define.h"

class Car
{
public:
	int id;
	int location;
	int status;
	int speed;
};
class Lane
{
public:
	bool dir;//��������
	int idLane;//����ID
	std::vector<Car> laneCar;//��¼ÿ�������ϵĳ�����Ϣ

};
class Road
{
public:
	Road();
	~Road();

	//����������Ϣ
	void CreateLane();

	int id;
	int length;
	int speed;
	int channel;
	int idFrom;
	int idTo;
	int isDuplex;
	Lane *lane;

	enum
	{// ��������
		FORWARD,	// ����
		BACKWARD,	// ����
	};
};

