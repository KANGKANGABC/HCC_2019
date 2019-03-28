#include "Algorithm.h"




Algorithm::Algorithm(DataCenter &dc)
{
	m_dc = &dc;
	num_Cars = dc.m_car_num;
	num_Roads = dc.m_road_num;
	num_Cross = dc.m_cross_num;
	cars = dc.car;
	roads = dc.road;
	crosses = dc.cross;
	graphC2R = dc.graphC2R;
	speedType = dc.speedType;
}


Algorithm::~Algorithm()
{
}

void Algorithm::ShortestTime_SpeedBasic_AutoPara()
{
	std::map<int, int> mapResult;
	int para = 80;
	Scheduler sd(*m_dc);

	getPath();
	for (int i = 0; i < 15; ++i)//����20��
	{
		getStartTime_BySpeed(para);
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 3;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	para = it->second;
	getStartTime_BySpeed(para);
	int time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);
}

void Algorithm::StaticAnalysis_SpeedBasic_AutoPara()
{
	std::map<int, int> mapResult;
	int para = 80;
	Scheduler sd(*m_dc);

	getStartTime_BySpeed(para);
	reorderCars();
	getPath_StaticAnalysis();
	for (int i = 0; i < 15; ++i)//����20��
	{
		getStartTime_BySpeed(para);
		reorderCars();
		getPath_StaticAnalysis();
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 3;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	para = it->second;
	getStartTime_BySpeed(para);
	reorderCars();
	getPath_StaticAnalysis();
	int time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);
}

void Algorithm::getPath()
{
	Graph_DG graph(m_dc->vexnum, m_dc->edge);
	graph.createArcGraph(m_dc->graphRoadLength);
	graph.createArcRoadvGraph(m_dc->graphRoadMaxSpeed);
	int timeCar = 0;
	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed, timeCar);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
		}
		cars[i].path = pathRoad;
		cars[i].time = timeCar;
	}
}

void Algorithm::getPath_StaticAnalysis()
{
	int num = 0;

	Graph_DG graph(m_dc->vexnum, m_dc->edge);
	graph.createArcGraph(m_dc->graphRoadLength);
	graph.createArcRoadvGraph(m_dc->graphRoadMaxSpeed);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.DijkstraNor(qCar[i].idCrossFrom, qCar[i].idCrossTo, qCar[i].speed);

		num++;
		//��ʱ���½�ͨӵ���ڽӾ���jamDegreeLongBefore
		if (num == 100)
		{
			num = 0;
			graph.upDateJamStatic();
			graph.cleanUpJamDegreeBefore();
		}

		//��ͳ�Ƶ�����ŵ� jamDegreeBefore�ľ�����
		for (int i = 0, j = 1; j < pathCross.size(); i++, j++)
		{
			graph.jamDegreeBefore[pathCross.at(i) - 1][pathCross.at(j) - 1]++;
		}

		graph.upDateJamDynamic();

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
		}

		qCar[i].path = pathRoad;
		cars[qCar[i].index].path = qCar[i].path;	//��qCar�õ���·����ֵ��cars��path������
	}
}

void Algorithm::getStartTime_BySpeed(int para)
{
	int n2, n4, n6, n8;
	n2 = para;
	n4 = para;
	n6 = para - para / 36;
	n8 = para - para / 12;

	for (int i = 0; i < num_Cars; ++i)//���Ե�0������
	{
		switch (cars[i].speed)
		{
		case 2:
			cars[i].starttime = 2 * n8 + 2 * n6 + 2 * n4 + i % (2 * n2);
			break;
		case 4:
			cars[i].starttime = 2 * n8 + 2 * n6 + i % (2 * n4);
			break;
		case 6:
			cars[i].starttime = 2 * n8 + i % (2 * n6);
			break;
		case 8:
			cars[i].starttime = i % (2 * n8);
			break;
		default:
			break;
		}

		if (cars[i].starttime < cars[i].plantime)
			cars[i].starttime = cars[i].plantime;
		cars[i].starttimeAnswer = cars[i].starttime;//starttimeAnswerΪ����д���ĳ���ʱ�䣬�������
	}
}

void Algorithm::swap(int i, int j)
{
	Car tmp;
	tmp = qCar[i];
	qCar[i] = qCar[j];
	qCar[j] = tmp;
}

void Algorithm::quicksort(int begin, int end)
{
	int i, j;
	i = begin + 1;
	j = end;
	if (begin < end)
	{
		while (i < j)
		{
			if (qCar[i].starttime > qCar[begin].starttime)
			{
				swap(i, j);
				j--;
			}
			else
				i++;
		}
		if (qCar[i].starttime > qCar[begin].starttime)
			i--;
		swap(i, begin);
		quicksort(begin, i - 1);
		quicksort(i + 1, end);
	}
}

void Algorithm::reorderCars()
{
	qCar.clear();
	for (int i = 0; i < num_Cars; i++)
	{
		qCar.push_back(cars[i]);//��id˳��ĳ����ŵ�qcar��vector��
	}
	int begin = 0;
	int end = qCar.size() - 1;
	quicksort(begin, end);
}

