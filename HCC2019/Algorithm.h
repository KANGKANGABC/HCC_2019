#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "define.h"
#include "DataCenter.h"
#include "dijkstra.h"
#include "Scheduler.h"

class Algorithm
{
//Vals
public:

private:
	DataCenter * m_dc;

	int num_Cars;
	int num_Roads;
	int num_Cross;
	Car *cars;//���еĳ��������������ָ�룩
	Road *roads;//���еĵ�·����·���������ָ�룩
	Cross *crosses;//���е�·�ڣ�·�ڶ��������ָ�룩

	//CrossToRoadת����
	std::vector<std::vector<int> > graphC2R;
	//�洢�������ٶ����������
	std::vector<int> speedType;
	//���ճ���ʱ�佫����������

	vector<Car> reorderCar;//���ճ���ʱ�佫����������
	vector<Car> qCar;	//�����ٶȽ�����������

//Functions
public:
	Algorithm(DataCenter &dc);
	~Algorithm();
	//·�������ʱ��·�� ʱ�䣺�����ٶȷ������� ���Σ��������Զ�����
	void ShortestTime_SpeedBasic_AutoPara();

	//·�������ʱ��·�� + ��̬·���Ż� ʱ�䣺�����ٶȷ������� ���Σ��������Զ�����
	void StaticAnalysis_SpeedBasic_AutoPara();

	//·�������ʱ��·�� + ��������̬�Ż� ʱ�䣺�����ٶȷ������� ���Σ��������Զ�����
	void DynamicPathByScheduler_SpeedBasic_AutoPara(int w);

	//·�������ʱ��·�� + ��̬·���Ż�����һ���� ʱ�䣺�����ٶȷ�������+ͬʱ��ͬ�ص�ֻ��һ�� ���Σ��������Զ�����
	void StaticAnalysisNor_SpeedBasicNoSame_AutoPara(int para);

	//·�������ʱ��·��  ʱ�䣺�����ٶȷ�������+����·����ѡ���ʵĳ� ���Σ��������Զ�����
	void ShortestTime_SpeedBasicRoadStatus_AutoPara(int para);

	//�޸�����
	void unlockDead(int para);

private:
	/*���·���ķ���*/
	void getPath();//�������·���㷨���·��
	void getPath_StaticAnalysis();//���·���㷨+��̬����

	/*��ó���ʱ��ķ���*/
	void getStartTime_BySpeed(int para);
	void ReOrderStartBySpeedAndStartCross(int para);//�����ٶȺͳ��������°��ų���ʱ��

	/*��������*/
	int getPartition(vector<Car> &reorderCar, int begin, int end);
	void quicksort(vector<Car> &reorderCar, int begin, int end);
	void quicksort(int begin, int end);
	void reorderCars(vector<Car> &reorderCar);
	void reorderCarsStarttime(vector<Car> &reorderCar);



};

#endif
