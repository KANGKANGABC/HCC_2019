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
	for (int i = 0; i < 15; ++i)//����15��
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
	for (int i = 0; i < 15; ++i)//����15��
	{
		getStartTime_BySpeed(para);
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
	getPath_StaticAnalysis();
	int time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);
}

void Algorithm::DynamicPathByScheduler_SpeedBasic_AutoPara(int w)
{
	std::map<int, int> mapResult;
	int para = 300;
	int time = 0;
	Scheduler sd(*m_dc);

	for (int i = 0; i < 15; ++i)//����15��
	{
		getPath();
		getStartTime_BySpeed(para);
		int time = sd.getSysTimeChangePath(w);
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
	getPath();
	getStartTime_BySpeed(para);
	time = sd.getSysTimeChangePath(w);
	PRINT("time:%d\n", time);
	time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);
}

void Algorithm::StaticAnalysisNor_SpeedBasicNoSame_AutoPara(int para)
{
	//���ݵ�ͼѡ����
	int paraFinal;
	int carLastArrive;
	int carTimeEarly;
	switch (cars[0].speed)				//ѡ��ͬ�ٶȵĿ�ʼ��������ֹ����ʱ��
	{
	case 8:  //��ͼ1
		carLastArrive = 15;
		carTimeEarly = 50;
		break;
	case 6:			//��ͼ2
		carLastArrive = 10;
		carTimeEarly =50;
		break;
	default:
		break;
	}
	std::map<int, int> mapResult;
	int time = 0;
	Scheduler sd(*m_dc);
	for (int i = 0; i < 15; ++i)//����15��
	{
		ReOrderStartBySpeedAndStartCross(para);
		reorderCarsStarttime();
		getPath_StaticAnalysis();
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -=1;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	it;
	para = it->second;
	ReOrderStartBySpeedAndStartCross(para);
	reorderCarsStarttime();
	getPath_StaticAnalysis();
	int timeFinal = sd.getSysTime();
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (timeFinal - carLastArrive))
		{
			cars[i].starttime = cars[i].starttime - carTimeEarly;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	time = sd.getSysTime();
	PRINT("time:%d\n", timeFinal);
	PRINT("timeFinal:%d\n", time);


	/*std::map<int, int> mapResult;
	int time = 0;
	Scheduler sd(*m_dc);
	for (int i = 0; i < 15; ++i)//����15��
	{
		ReOrderStartBySpeedAndStartCross(para);
		reorderCarsStarttime();
		getPath_StaticAnalysis();
		int time = sd.getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time, para));
		para -=1;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first, v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	it;
	para = it->second;
	ReOrderStartBySpeedAndStartCross(para);
	reorderCarsStarttime();
	getPath_StaticAnalysis();
	int timeFinal = sd.getSysTime();
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (timeFinal -10))
		{
				cars[i].starttime = cars[i].starttime -50;
				cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	time = sd.getSysTime();
	PRINT("time:%d\n", timeFinal);
	PRINT("timeFinal:%d\n", time);
	*/
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
	for (int i = 0; i < 15; ++i)//����20��
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
	//�޸��������ĳ���ʱ��
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 60;//����ʱ���ذ���
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//������һ�¿��ǲ�������
	PRINT("timeUnlock1:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//����ʱ���ذ���
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//������һ�¿��ǲ�������
	PRINT("timeUnlock2:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//����ʱ���ذ���
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//������һ�¿��ǲ�������
	PRINT("timeUnlock3:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//����ʱ���ذ���
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//������һ�¿��ǲ�������
	PRINT("timeUnlock4:%d\n", time);
	for (int i = 0; i < sd.carsDeadLock.size() / 2; ++i)
	{
		Car car = sd.carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//����ʱ���ذ���
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = sd.getSysTimeChangePath(w);//������һ�¿��ǲ�������
	PRINT("timeUnlock5:%d\n", time);
	time = sd.getSysTime();
	PRINT("timeFinal:%d\n", time);

	sd.carsDeadLock.clear();//�����������

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

bool Comp(const int &a, const int &b);

void Algorithm::ReOrderStartBySpeedAndStartCross(int para)
{
	int early2,early4,early6,early8;
	int delay;
	switch (cars[0].speed)				//ѡ��ͬ�ٶȵĿ�ʼ��������ֹ����ʱ��
	{
	case 8:  //��ͼ1
		early2 =0;
		early4 =1;
		early6 = para/36;
		early8 =para/12;
		delay =10;
		break;
	case 6:			//��ͼ2
		early2 = 0;
		early4 = 0;
		early6 = 2;
		early8 = 8;
		delay =10;
		break;
	default:
		break;
	}

	int n2, n4, n6, n8;
	n2 = para-early2;
	n4 = para-early4;
	n6 = para-early6;
	n8 = para-early8;

	sort(speedType.begin(), speedType.end(), Comp);  //speedType�������ٶ�����

	for (auto speed : speedType)
	{
		std::queue<Car> qspeed;		//��car��speed˳�����������qspeed

		assert(qspeed.size() == 0);		//ȷ��qspeedΪ��

		for (int i = 1; i <= num_Cars; ++i)			//��car���ٶ��Ⱥ�����У�ÿ�ζ�����ֻ��һ���ٶȵĳ�
		{
			if (cars[i - 1].speed == speed)
			{
				qspeed.push(cars[i - 1]);
			}
		}

		int timebegin, timeend;
		switch (speed)				//ѡ��ͬ�ٶȵĿ�ʼ��������ֹ����ʱ��
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
			timebegin = 2 * (n8 + n6 + n4) + delay;
			timeend = 2 * (n8 + n6 + n4 + n2) +delay;
			break;
		default:
			break;
		}

		int carStartPerSec = qspeed.size() / (timeend - timebegin) + 1;  //����ÿʱ��Ƭ��Ҫ�����ĳ���������һ��Ϊ�˷�ֹ�����꣬���ܻᵼ�²�ͬ�ٶȷ������μ��м��
		for (int time = timebegin; time <= timeend; time++)		//ͬһʱ��Ƭͬһ�ص�ĳ�ֻ��һ��
		{
			if (qspeed.empty())			//���qspeed���ˣ�˵�����ٶȵĳ��������ˣ��˳���ǰѭ��
				break;

			vector <int>fromCross;				//���ڴ洢ÿһʱ��Ƭ�ѷ��䳵���ĳ����أ�ÿһʱ��Ƭ��Ҫ��ʼ��Ϊ0
			fromCross.resize(num_Cross);
			for (int i = 0; i < num_Cross; i++)
			{
				fromCross[i] = 0;
			}

			for (int carAssigned = 0; carAssigned < carStartPerSec; carAssigned++)
			{
				//�����еĳ�����ĿС��carStartPerSec
				if (qspeed.empty())			//���qspeed���ˣ�˵�����ٶȵĳ��������ˣ��˳���ǰѭ��
					break;

				//�����л��пɷ��ĳ�
				bool carIsAssigned = false;
				int frequence = 0;			//���ڼ�¼ѭ�����д���Ҳ�����������ж��ٳ������ʹ�
				while (carIsAssigned == false)			//���������һ�������������������һ����
				{
					++frequence;
					int carOrder = qspeed.front().index;        //���ڼ�¼���׵�car���±�

					if (qspeed.front().plantime <= time && fromCross[qspeed.front().idCrossFrom - 1] < 1)
					{
						//�����׵�car��plantimeС�ڵ��ڵ�ǰʱ���Ҹó�����ֻ��һ�����򽫸ó�starttimeAnswer��Ϊ�˿̣�����qspeed�����е�����carIsAssigned��Ϊtrue��fromCross��ǰ�����ؼ�һ
						cars[carOrder].starttimeAnswer = time;
						cars[carOrder].starttime = time;
						fromCross[qspeed.front().idCrossFrom - 1] ++;
						qspeed.pop();
						carIsAssigned = true;
					}
					else if (qspeed.front().plantime > time)		//plantime��time֮���������ͬ�����صĳ������˵������ǰ��ֱ�������ţ�����Ҫ���Ƿ������һ�ζ�����
					{
						qspeed.pop();
						qspeed.push(cars[carOrder]);
					}
					else      //������ͬ�����صĳ������˵����������ʱû�б�����һ�ζ����ˣ�����������������·���ʱ�䣬��ʱ���в�ֹһ�����ӳ����ط���
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
		reorderCar.push_back(cars[i]);//��id˳��ĳ����ŵ�qcar��vector��
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
		qCar.push_back(cars[i]);//��id˳��ĳ����ŵ�qcar��vector��
	}

	int begin = 0;
	int end = qCar.size() - 1;
	quicksort(begin, end);
}
