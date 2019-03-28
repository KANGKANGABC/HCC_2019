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
	void InitDijkstra();//Dijkstra算法初始化
	void getPath();//基于最短路径算法获得路径

};

#endif
