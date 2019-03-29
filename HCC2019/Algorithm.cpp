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
	reorderCars(reorderCar);
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
	for (int i = 0; i < 15; ++i)//迭代15次
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
	getPath_StaticAnalysis();
	for (int i = 0; i < 15; ++i)//迭代15次
	{
		getStartTime_BySpeed(para);
		reorderCarsStarttime();
		getPath_StaticAnalysis();
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 2;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	para = it->second;
	getStartTime_BySpeed(para);
	reorderCarsStarttime();
	getPath_StaticAnalysis();
	int time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);
}

void Algorithm::DynamicPathByScheduler_SpeedBasic_AutoPara(int w)
{
	std::map<int, int> mapResult;
	int para = 55;
	int time = 0;
	Scheduler sd(*m_dc);

	for (int i = 0; i < 15; ++i)//迭代15次
	{
		getPath();
		getStartTime_BySpeed(para);
		reorderCarsStarttime();
		int time = sd.getSysTimeChangePath(w);
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 1;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	para = it->second;
	getPath();
	getStartTime_BySpeed(para);
	reorderCarsStarttime();
	time = sd.getSysTimeChangePath(w);
	PRINT("time:%d\n", time);
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (time - 20))
		{
			cars[i].starttime = cars[i].starttime - 30;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	time = sd.getSysTime();
	int timeV2 = sd.getSysTimeV2();
	PRINT("timeFinal:V0:%d   V2:%d\n", time, timeV2);
}

void Algorithm::StaticAnalysisNor_SpeedBasicNoSame_AutoPara(int para)
{
	std::map<int, int> mapResult;
	int time = 0;
	Scheduler sd(*m_dc);
	for (int i = 0; i < 15; ++i)//迭代15次
	{
		ReOrderStartBySpeedAndStartCross(para);
		reorderCarsStarttime();
		getPath_StaticAnalysis();
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 2;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	it++;
	para = it->second;
	ReOrderStartBySpeedAndStartCross(para);
	reorderCarsStarttime();
	getPath_StaticAnalysis();
	int timeFinal = sd.getSysTime();
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (timeFinal - 20))
		{
			cars[i].starttime = cars[i].starttime - 20;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	time = sd.getSysTime();
	int timeV2 = sd.getSysTimeV2();
	PRINT("timeFinal:V0:%d   V2:%d\n", time, timeV2);
}

void Algorithm::ShortestTime_SpeedBasicRoadStatus_AutoPara(int para)
{
	Scheduler sd(*m_dc);
	int time = sd.getSysTimeChangeTime(0);
	PRINT("time:%d\n",time);
}

void Algorithm::unlockDead(int para)
{
	std::map<int, int> mapResult;
	int timeMax = INT_MAX;
	int time, timeFinal;
	int w = 9;
	Scheduler sd(*m_dc);
	for (int i = 0; i < 15; ++i)//迭代20次
	{
		getStartTime_BySpeed(para);
		getPath();
		int time = sd.getSysTimeChangePath(w);
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 4;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	para = it->second;

	para = para - 4;
	getStartTime_BySpeed(para);
	getPath();
	time = sd.getSysTimeChangePath(w);
	//修改死锁车的出发时间
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 60;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock1:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock2:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock3:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock4:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock5:%d\n", time);
	time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);

	sd.carsDeadLock.clear();//清空死锁队列

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
		//定时更新交通拥堵邻接矩阵jamDegreeLongBefore
		if (num == 100)
		{
			num = 0;
			graph.upDateJamStatic();
			graph.cleanUpJamDegreeBefore();
		}

		//将统计的情况放到 jamDegreeBefore的矩阵中
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
		cars[qCar[i].index].path = qCar[i].path;	//将qCar得到的路径赋值到cars的path变量中
	}
}

void Algorithm::getStartTime_BySpeed(int para)
{
	int n2, n4, n6, n8;
	n2 = para;
	n4 = para;
	n6 = para - para / 36;
	n8 = para - para / 12;

	for (int i = 0; i < num_Cars; ++i)//忽略第0行数据
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
		cars[i].starttimeAnswer = cars[i].starttime;//starttimeAnswer为最终写出的出发时间，不会更改
	}
}

bool Comp(const int &a, const int &b);

void Algorithm::ReOrderStartBySpeedAndStartCross(int para)
{
	int n2, n4, n6, n8;
	n2 = para;
	n4 = para;
	n6 = para;
	n8 = para;

	sort(speedType.begin(), speedType.end(), Comp);  //speedType里存的是速度类型

	for (auto speed : speedType)
	{
		std::queue<Car> qspeed;		//将car按speed顺序依次入队列qspeed

		assert(qspeed.size() == 0);		//确保qspeed为空

		for (int i = 1; i <= num_Cars; ++i)			//将car按速度先后入队列，每次队列里只有一种速度的车
		{
			if (cars[i - 1].speed == speed)
			{
				qspeed.push(cars[i - 1]);
			}
		}

		int timebegin, timeend;
		switch (speed)				//选择不同速度的开始发车和终止发车时刻
		{
		case 8:
			timebegin = 1;
			timeend = 2 * n8;
			break;
		case 6:
			timebegin = 2 * n8 + 1;
			timeend = 2 * (n8 + n6);
			break;
		case 4:
			timebegin = 2 * (n8 + n6) + 1;
			timeend = 2 * (n8 + n6 + n4);
			break;
		case 2:
			timebegin = 2 * (n8 + n6 + n4) + 1;
			timeend = 2 * (n8 + n6 + n4 + n2);
			break;
		default:
			break;
		}

		int carStartPerSec = qspeed.size() / (timeend - timebegin) + 1;  //计算每时间片需要发出的车数量，加一是为了防止发不完，可能会导致不同速度发车批次间有间隔
		for (int time = timebegin; time <= timeend; time++)		//同一时间片同一地点的车只发一辆
		{
			if (qspeed.empty())			//如果qspeed空了，说明该速度的车分配完了，退出当前循环
				break;

			vector <int>fromCross;				//用于存储每一时间片已分配车辆的出发地，每一时间片都要初始化为0
			fromCross.resize(num_Cross);
			for (int i = 0; i < num_Cross; i++)
			{
				fromCross[i] = 0;
			}

			for (int carAssigned = 0; carAssigned < carStartPerSec; carAssigned++)
			{
				//队列中的车辆数目小于carStartPerSec
				if (qspeed.empty())			//如果qspeed空了，说明该速度的车分配完了，退出当前循环
					break;

				//队列中还有可发的车
				bool carIsAssigned = false;
				int frequence = 0;			//用于记录循环进行次数也即队列中已有多少车被访问过
				while (carIsAssigned == false)			//如果安排了一辆车，则结束，安排下一辆车
				{
					++frequence;
					int carOrder = qspeed.front().index;        //用于记录队首的car的下标

					if (qspeed.front().plantime <= time && fromCross[qspeed.front().idCrossFrom - 1] < 1)
					{
						//若队首的car的plantime小于等于当前时刻且该出发地只有一辆，则将该车starttimeAnswer设为此刻，并从qspeed队列中弹出，carIsAssigned设为true，fromCross当前出发地加一
						cars[carOrder].starttimeAnswer = time;
						cars[carOrder].starttime = time;
						fromCross[qspeed.front().idCrossFrom - 1] ++;
						qspeed.pop();
						carIsAssigned = true;
					}
					else if (qspeed.front().plantime > time)		//plantime在time之后或已有相同出发地的车发出了的情况，前者直接往后排，后者要看是否遍历完一次队列了
					{
						qspeed.pop();
						qspeed.push(cars[carOrder]);
					}
					else      //已有相同出发地的车发出了的情况，若此时没有遍历完一次队列了，则继续遍历，否则定下发车时间，此时会有不止一辆车从出发地发出
					{
						if (frequence < qspeed.size())
						{
							qspeed.pop();
							qspeed.push(cars[carOrder]);
						}
						else
						{
							cars[carOrder].starttimeAnswer = time;
							cars[carOrder].starttime = time;
							fromCross[qspeed.front().idCrossFrom - 1] ++;
							qspeed.pop();
							carIsAssigned = true;
						}
					}
				}
			}
		}
	}
}

int Algorithm::getPartition(vector<Car> &reorderCar, int begin, int end)
{
	Car keyVal;
	keyVal = reorderCar[begin];
	while (begin < end)
	{
		while (begin < end && reorderCar[end].speed >= keyVal.speed)
			end--;
		reorderCar[begin] = reorderCar[end];
		while (begin < end && reorderCar[begin].speed <= keyVal.speed)
			begin++;
		reorderCar[end] = reorderCar[begin];
	}
	reorderCar[begin] = keyVal;
	return begin;
}

void Algorithm::quicksort(vector<Car> &reorderCar, int begin, int end)
{
	stack<int> s;
	if (begin < end)
	{
		int mid = getPartition(reorderCar, begin, end);
		if (mid - 1 > begin)
		{
			s.push(begin);
			s.push(mid - 1);
		}
		if (mid + 1 < end)
		{
			s.push(mid + 1);
			s.push(end);
		}

		while (!s.empty())
		{
			int qHeight = s.top();
			s.pop();
			int pLow = s.top();
			s.pop();
			int pqMid = getPartition(reorderCar, pLow, qHeight);
			if (pqMid - 1 > pLow)
			{
				s.push(pLow);
				s.push(pqMid - 1);
			}
			if (pqMid + 1 < qHeight)
			{
				s.push(pqMid + 1);
				s.push(qHeight);
			}
		}
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

bool less_speed(const Car & m1, const Car & m2) {
	return m1.speed < m2.speed;
}

void Algorithm::reorderCars(vector<Car> &reorderCar)
{
	reorderCar.clear();

	for (int i = 0; i < num_Cars; i++)
	{
		reorderCar.push_back(cars[i]);//将id顺序的车辆放到qcar的vector中
	}

	int begin = 0;
	int end = reorderCar.size() - 1;
	//quicksort(reorderCar, begin, end);
	std::sort(reorderCar.begin(), reorderCar.end(),less_speed);
}

bool less_starttime(const Car & m1, const Car & m2) {
	return m1.starttime < m2.starttime;
}

void Algorithm::reorderCarsStarttime()
{
	qCar.clear();

	for (int i = 0; i < num_Cars; i++)
	{
		qCar.push_back(cars[i]);//将id顺序的车辆放到qcar的vector中
	}

	int begin = 0;
	int end = qCar.size() - 1;
	quicksort(begin, end);
}
