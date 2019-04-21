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
	mapId2IndexCar = dc.mapId2IndexCar;
	mapId2IndexCross = dc.mapId2IndexCross;
	mapId2IndexRoad = dc.mapId2IndexRoad;
	num_path_preset = dc.m_path_preset_num;
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
	int para = 300;
	Scheduler sd(*m_dc);

	getStartTime_BySpeed(para);
	reorderCarsStarttime();
	getPath_StaticAnalysis();
	for (int i = 0; i < 5; ++i)//迭代15次
	{
		getStartTime_BySpeed(para);
		reorderCarsStarttime();
		getPath_StaticAnalysis();
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -= 10;
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
	int timeV2 = sd.getSysTimeV2();
	PRINT("timeFinal:V0:%d   V2:%d\n", time, timeV2);
}

void Algorithm::DynamicPathByScheduler_SpeedBasic_AutoPara(int w)
{
	std::map<int, int> mapResult;
	int para = 350;
	int time = 0;
	Scheduler sd(*m_dc);


	//getStartTime_BySpeed(para);
	//reorderCarsStarttime();

	for (int i = 0; i < 20; ++i)//迭代15次
	{
		getStartTime_BySpeed(para);
		int time = sd.getSysTimeChangePath(w);
		if (time == false)
		{
			PRINT("DeadLock para:%d\n",para);
			time = INT_MAX;
		}
		else
		{
			PRINT("Good para:%d time:%d\n", para,time);
		}
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

	/*
	getStartTime_BySpeed(para);
	reorderCarsStarttime();
	*/
	
	getStartTime_BySpeed(para);


	time = sd.getSysTimeChangePath(w);
	PRINT("time:%d\n", time);
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (time - 20))
		{
			cars[i].starttime = cars[i].starttime - 20;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	//time = sd.getSysTime();
	int timeV2 = sd.getSysTimeV2();

	PRINT("timeFinal:V0:%d   V2:%d\n", time, timeV2);
}

void Algorithm::StaticAnalysisNor_SpeedBasicNoSame_AutoPara(int para)
{

	std::map<int, int> mapResult;
	int time = 0;
	int timeV2 = 0;
	Scheduler sd(*m_dc);

	int paraFinal;
	int carLastArrive;
	int carTimeEarly;
	int timePresetLast;
	switch (cars[1].speed)				//选择不同速度的开始发车和终止发车时刻
	{
	case 14:
		paraFinal = 120;
		carLastArrive = 0;
		carTimeEarly = 0;
		timePresetLast = 323;
		break;
	case 8:	
		paraFinal = 135;
		carLastArrive = 0;
		carTimeEarly = 0;
		timePresetLast = 223;
		break;
	default:
		break;
	}

	ReOrderStartBySpeedAndStartCross(paraFinal);
	reorderCarsStarttime();
	getPath_StaticAnalysisNor();
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].preset != 1)
		{
			cars[i].starttime = cars[i].starttime + timePresetLast + 10;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	vector<Car> carsChangeTime;
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].starttime > timePresetLast && cars[i].starttime < timePresetLast + timePresetLast/2)
		{
			carsChangeTime.push_back(cars[i]);
		}
	}
	printf("carsChangeTime.size:%d\n", carsChangeTime.size());
	for (auto car : carsChangeTime)
	{
		int startTime = rand() % (timePresetLast);
		if (startTime >= car.plantime)
		{
			//修改plantime
			cars[car.index].starttime = startTime;
			cars[car.index].starttimeAnswer = startTime;
		}
		else
		{
			cars[car.index].starttime = car.plantime;
			cars[car.index].starttimeAnswer = startTime;
		}
	}
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].preset != 1&& cars[i].starttime > timePresetLast)
		{
			cars[i].starttime = cars[i].starttime - timePresetLast/2;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	//time = sd.getSysTime();
	PRINT("timeFinal:V0:%d\n", time);
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

void Algorithm::tryUnlockDead(Scheduler sd,int para)
{
	int count = 5;//尝试解锁5次
	while (count > 0)
	{

		count--;
	}
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
		vector<int> pathCross = graph.Dijkstra(qCar[i].idCrossFrom, qCar[i].idCrossTo, qCar[i].speed);

		num++;
		//定时更新交通拥堵邻接矩阵jamDegreeLongBefore
		if (num == 200)
		{
			num = 0;
			graph.upDateJam();
		}

		//将统计的情况放到 jamDegreeBefore的矩阵中
		for (int i = 0, j = 1; j < pathCross.size(); i++, j++)
		{
			graph.jamDegreeTmp[pathCross.at(i)][pathCross.at(j)]++;
		}

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j]][pathCross[j + 1]];
			//assert(pathRoad[j] != 0);
		}

		qCar[i].path = pathRoad;

		cars[qCar[i].index].path = qCar[i].path;		//将qCar得到的路径赋值到cars的path变量中
	}
}

void Algorithm::getPath_StaticAnalysisNor()
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
			graph.jamDegreeBefore[pathCross.at(i)][pathCross.at(j)]++;
		}

		graph.upDateJamDynamic();

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j]][pathCross[j + 1]];
		}
		if (cars[qCar[i].index].preset != 1)
		{
			qCar[i].path = pathRoad;
			cars[qCar[i].index].path = qCar[i].path;	//将qCar得到的路径赋值到cars的path变量中
		}

	}
	int numCanBeChange = num_path_preset / 10;
	printf("numCanBeChange:%d\n", numCanBeChange);
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].preset == 1)
		{
			vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo);
			vector<int> pathRoadDJ(pathCross.size() - 1);
			for (int j = 0; j < pathRoadDJ.size(); ++j)
			{
				pathRoadDJ[j] = graphC2R[pathCross[j]][pathCross[j + 1]];
			}
			if ((pathRoadDJ.size() + 3) < cars[i].path.size())
			{
				cars[i].path = pathRoadDJ;
				cars[i].isChanged = true;
				numCanBeChange--;
			}
		}
		if (numCanBeChange <= 0)
			break;
	}
}

void Algorithm::getStartTime_BySpeed(int para)
{
	int n4 = para;
	int n6 = para;
	int n8 = para;
	int n10 = para;
	int n12 = para;
	int n14 = para;
	int n16 = para;


	for (int i = 0; i < num_Cars; ++i)//忽略第0行数据
	{
		switch (cars[i].speed)				//选择不同速度的开始发车和终止发车时刻
		{
		case 16:
			cars[i].starttime = rand() % (2 * para);
			break;
		case 14:
			cars[i].starttime = 2 * n16 + rand() % (2 * para);
			break;
		case 12:
			cars[i].starttime = 2 * (n16 + n14) + rand() % (2 * para);
			break;
		case 10:
			cars[i].starttime = 2 * (n16 + n14 + n12) + rand() % (2 * para);
			break;
		case 8:
			cars[i].starttime = 2 * (n16 + n14 + n12 + n10) + rand() % (2 * para);
			break;
		case 6:
			cars[i].starttime = 2 * (n16 + n14 + n12 + n10 + n8) + rand() % (2 * para);
			break;
		case 4:
			cars[i].starttime = 2 * (n16 + n14 + n12 + n10 + n8 + n6) + rand() % (2 * para);
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
	int early4=0,early6=0,early8=0, early10 = 0, early12 = 0, early14=0, early16= 0;
	int delay=0;

	switch (cars[0].speed)				//选择不同速度的开始发车和终止发车时刻
	{
	case 10:  //地图1
		early4 =0;
		early6 = 0;
		early8 =0;
		early10 =0;
		early12 =0;
		early14 =0;
		early16 =0;
		delay =0;
		break;
	case 6:			//地图2
		early4 = 0;
		early6 = 0;
		early8 = 0;
		early10 = 0;
		early12 = 0;
		early14 = 0;
		early16 = 0;
		delay = 0;
		break;
	default:
		break;
	}
	
	int n4, n6, n8,n10,n12,n14,n16;
	n4 = para-early4;
	n6 = para-early6;
	n8 = para-early8;
	n10 = para - early10;
	n12 = para - early12;
	n14 = para - early14;
	n16 = para - early16;

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
		case 16:
			timebegin = 1;
			timeend = 2 * n16;
			break;
		case 14:
			timebegin = 2 * n16 + 1;
			timeend = 2 * (n16 + n14);
			break;
		case 12:
			timebegin = 2 * (n16 + n14) + 1;
			timeend = 2 * (n16 + n14 + n12);
			break;
		case 10:
			if (para == 122)
			{
				timebegin = 2 * (n16 + n14 + n12);
			}
			else
			{
				timebegin = 2 * (n16 + n14 + n12) + 1;
			}
			
			timeend = 2 * (n16+ n14 + n12 + n10) ;
			break;
		case 8:
			timebegin = 2 * (n16 + n14 + n12 + n10) +1;
			timeend = 2 * (n16 + n14 + n12 + n10+n8) ;
			break;
		case 6:
			timebegin = 2 * (n16 + n14 + n12 + n10 + n8)+1 ;
			timeend = 2 * (n16 + n14 + n12 + n10 + n8+n6) ;
			break;
		case 4:
			timebegin = 2 * (n16 + n14 + n12 + n10 + n8 + n6) + delay;
			timeend = 2 * (n16 + n14 + n12 + n10 + n8 + n6+n4) + delay;
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

					if (qspeed.front().plantime <= time && fromCross[id2indexCross(qspeed.front().idCrossFrom)] < 1)
					{
						//若队首的car的plantime小于等于当前时刻且该出发地只有一辆，则将该车starttimeAnswer设为此刻，并从qspeed队列中弹出，carIsAssigned设为true，fromCross当前出发地加一
						if (cars[carOrder].preset == 1)
						{
							cars[carOrder].starttimeAnswer = cars[carOrder].starttime;
						}
						else
						{
							cars[carOrder].starttimeAnswer = time;
							cars[carOrder].starttime = time;
						}
						fromCross[id2indexCross(qspeed.front().idCrossFrom)] ++;
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
							if (cars[carOrder].preset == 1)
							{
								cars[carOrder].starttimeAnswer = cars[carOrder].starttime;
							}
							else
							{
								cars[carOrder].starttimeAnswer = time;
								cars[carOrder].starttime = time;
							}
							fromCross[id2indexCross(qspeed.front().idCrossFrom)] ++;
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
	//std::sort(qCar.begin(), qCar.end(), less_starttime);
	quicksort(begin, end);
}

int Algorithm::id2indexCar(int id)
{
	map<int, int>::iterator it;
	it = mapId2IndexCar.find(id);
	assert(it != mapId2IndexCar.end());
	return it->second;
}

int Algorithm::id2indexRoad(int id)
{
	map<int, int>::iterator it;
	it = mapId2IndexRoad.find(id);
	assert(it != mapId2IndexRoad.end());
	return it->second;
}

int Algorithm::id2indexCross(int id)
{
	map<int, int>::iterator it;
	it = mapId2IndexCross.find(id);
	assert(it != mapId2IndexCross.end());
	return it->second;
}

