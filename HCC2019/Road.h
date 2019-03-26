#pragma once
#include "define.h"

class Car
{
public:
	int id;
	int location;
	int status;
	int speed;
	int plantime;//�ƻ�ʻ��ʱ��
	int starttime;//��ʵʻ��ʱ��
	int starttimeAnswer;//д��answer��starttime
	int timeArrived;//��¼��������ʱ��
	int idCurRoad;//��ǰ�����ڵĵ�·ID
	int idCurLane;//��ǰ�����ڵĳ���ID
	int dirCross;//��־��·�ڵ�״̬ 
	int idCrossFrom;//���ĳ���·��
	int idCrossTo;//������ֹ·��
	int time;
	int dirMap;//���ڵ�ͼ�ϵķ��򣬷�Ϊ��++����--����+-����-+������
	std::vector<int> path;
}; 

class Cross
{
public:
	int id;
	int roadID_T; //˳ʱ���1�� Top
	int roadID_R; //˳ʱ���2�� Right
	int roadID_D; //˳ʱ���3�� Down
	int roadID_L; //˳ʱ���4�� Left
	std::vector<int> roadID;//��˳��洢�����ĸ����򣬱��ڱ���

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
};

