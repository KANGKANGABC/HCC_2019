#ifndef ALGORITHM_H_
#define ALGORITHM_H_

#include "define.h"
#include "DataCenter.h"

class Algorithm
{
//Vals
public:

private:
	DataCenter * m_dc;
//Functions
public:
	Algorithm(DataCenter &dc);
	~Algorithm();

private:
	void InitDijkstra();//Dijkstra�㷨��ʼ��
	void getPath();//�������·���㷨���·��

};

#endif
