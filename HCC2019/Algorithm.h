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
	vector<Car> qCar;

//Functions
public:
	Algorithm(DataCenter &dc);
	~Algorithm();
	//·�������ʱ��·�� ʱ�䣺�����ٶȷ������� ���Σ��������Զ�����
	//ShortestTime_SpeedBasic_AutoPara
	void ShortestTime_SpeedBasic_AutoPara();
	//·�������ʱ��·�� + ��̬·���Ż� ʱ�䣺�����ٶȷ������� ���Σ��������Զ�����
	//StaticAnalysis_SpeedBasic_AutoPara
	void StaticAnalysis_SpeedBasic_AutoPara();


private:
	/*���·���ķ���*/
	void getPath();//�������·���㷨���·��
	void getPath_StaticAnalysis();//���·���㷨+��̬����

	/*��ó���ʱ��ķ���*/
	void getStartTime_BySpeed(int para);

	/*��������*/
	void swap(int i, int j);
	void quicksort(int begin, int end);
	void reorderCars();



};

#endif
