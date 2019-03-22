#pragma once
#include "define.h"
#include "Road.h"
#include "DataCenter.h"
#include "dijkstra.h"

class DataCenter;

class Scheduler
{
public:
	Scheduler(DataCenter &dc);
	~Scheduler();
	int getSysTime();
	//���ڶ�̬�������滮·��
	void getPathByScheduler();
	//���·��,Ϊÿ�����滮·��
	void getPath();
	//���·��,Ϊÿ�����滮·��������ʱ��
	void getPathByTime();
	void getPathByTime_dynamic(); //����1-100 ��101-199���Ĺ켣�����µ�200�������ڽӾ���

	int vexnum, edge;
	std::vector<std::vector<int> > tmp;
	std::vector<std::vector<int> > tmp1;
private:
	int num_CarsScheduling;//���ڵ��ȵ�car����
	int num_CarsPut;//�Ѿ�������car����
	int num_Roads;//��·����
	int num_Crosses;//·������
	int num_Cars;//������
	int time_Scheduler;//����������ʱ��
	Road *roads;//���еĵ�·����·���������ָ�룩
	Cross *crosses;//���е�·�ڣ�·�ڶ��������ָ�룩
	Car *cars;//���еĳ�

	std::deque<Car> carsWaitInGarage;//��һʱ��Ƭδʻ�����ȴ�ʻ���ĳ�

	//CrossToRoadת����
	std::vector<std::vector<int> > graphC2R;

	//��̬��������������ĵ�·�������
	std::vector<std::vector<float> > graphRoadStatusByDS;

	/*����road�ϵĳ����н���ֱ���ó�����ʻ��ɵȴ�״̬������ֹ״̬*/
	void driveAllCarsJustOnRoadToEndState();
	
	/*�øó�ǰ��*/
	int driveCar(Car car, int indexCar);//indexCarΪ�ó��ڳ�����λ��

	int driveCarNew(Car car);

	/*���ó������·��ʻ*/
	void addCar(Car car,int i);//iΪ�ó���cars[]�е��±꣬���ڼ��복ʧ��ʱ�Ӻ�ʱ��Ƭ

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

	//�����еĳ�����·��ʻ
	void driverCarInGarage();

	//�����еĳ�����·��ʻ,��̬������·��
	void driverCarInGarageDynamic(Graph_DG &graph);

	//��ӡ����״̬
	void putCarStatus(Car car);

	//������г���״̬
	void putAllCarStatus();

	//������е�·״̬
	void putAllRoadStatus();

	//��ø�cross�Ķ�Ӧ���ȼ���·ID�������·IDΪ-1�򷵻�-1
	int getFirstRoadFromCross(int idCross,int index);

	//���ĳroad��ĳcross��ĳ����
	int getDirByRoadCrossDir(int idCross,int idRoad);

	//ĳ��road�ϵ�ĳ��channel�����н���������ó�������ʻ���ܵ�����ֹ״̬�ĳ�
	void driveAllCarsJustOnOneChannelToEndState(int idRoad, int idCross, int idChannel);

};

