#pragma once
#include "define.h"
#include "Road.h"
#include "DataCenter.h"

class DataCenter;

class Scheduler
{
public:
	Scheduler(DataCenter &dc);
	~Scheduler();
	int getSysTime();
	//���·��,Ϊÿ�����滮·��
	void getPath();
private:
	int num_CarsScheduling;//���ڵ��ȵ�car����
	int num_Roads;//��·����
	int num_Crosses;//·������
	int num_Cars;//������
	int time_Scheduler;//����������ʱ��
	Road *roads;//���еĵ�·����·���������ָ�룩
	Cross *crosses;//���е�·�ڣ�·�ڶ��������ָ�룩
	Car *cars;//���еĳ�

	/*����road�ϵĳ����н���ֱ���ó�����ʻ��ɵȴ�״̬������ֹ״̬*/
	void driveAllCarsJustOnRoadToEndState();
	
	/*�øó�ǰ��*/
	void driveCar(Car car, int indexCar);//indexCarΪ�ó��ڳ�����λ��

	/*���ó������·��ʻ*/
	void addCar(Car car);

	//�ж�ĳcross��ĳroad�Ƿ������ʻ����//����IDΪ����ǰ��ID
	int isCanEnter(int idRoad, int idCross);//�������ֵ-1�������ɼ��룬���򷵻ؿ�ʻ���lane ID
		
	//�ж�ĳcross��ĳroad�Ƿ������Ҫֱ�еĳ�
	bool isBeDD(int idRoad, int idCross);

	//�ж�ĳcross��ĳroad�Ƿ������Ҫ��ת�ĳ�
	bool isBeLEFT(int idRoad, int idCross);

	//���뵱ǰ��·����һ��·���Լ�·��ID������������ӵ�ǰ��·ʻ����һ��·�ķ���
	int getCrossDir(int idCurRoad, int idNextRoad, int idNextCross);

	//��������ӵ�ǰ��·ʻ����һ·�ڣ���Ϊ��·���ٿ���������������ݹٷ��Ĺ�����㳵������ʻ��������
	//�������ʻ��������Ϊ0����ôֻ��ͣ��·�ڣ��ȴ���һ�ε���
	int getCrossDistance(Car car, int idCurRoad, int idNextRoad);

	//���ó���ʻ���¸�road ���ݼ���AA:��ʱ�������г������յ�
	void driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location);

	//�жϸó��ܷ���ĳ·��ת����ʻ
	bool isCanDriveToNextRoad(Car car, int dir, int idCross);//dirΪĿ����ʻ����

	//ĳ��road�ϵĳ����н���ֱ���ó�����ʻ��ɵȴ�״̬������ֹ״̬
	void driveAllCarsJustOnOneRoadToEndState(int idRoad, int idCross);

	//�����еĳ�����·��ʻ
	void driverCarInGarage();

	//��ӡ����״̬
	void putCarStatus(Car car);

	//������г���״̬
	void putAllCarStatus();

};

