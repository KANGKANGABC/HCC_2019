#pragma once
#include "define.h"
class Cross
{
public:
	int id;
	std::vector<Car> crossCar;//��¼Ŀǰ��·�ڵȴ����ȵĳ�����Ϣ
	int roadID_T; //˳ʱ���1�� Top
	int roadID_R; //˳ʱ���2�� Right
	int roadID_D; //˳ʱ���3�� Down
	int roadID_L; //˳ʱ���4�� Left

};
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

