#include "Algorithm.h"



Algorithm::Algorithm(DataCenter &dc)
{
	m_dc = &dc;
}


Algorithm::~Algorithm()
{
}

void Algorithm::InitDijkstra()
{
	Graph_DG graph(m_dc->vexnum, m_dc->edge);
	graph.createArcGraph(m_dc->graphRoadLength);
	graph.createArcRoadvGraph(m_dc->graphRoadMaxSpeed);
}

void Algorithm::getPath()
{

}
