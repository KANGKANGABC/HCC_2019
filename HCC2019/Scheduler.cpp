#include "Scheduler.h"
#include "fstream"
#include<string>

Scheduler::Scheduler(DataCenter &dc)
{
	num_CarsScheduling = dc.m_car_num;//获得需要调度的车数量
	num_Roads = dc.m_road_num;
	num_Crosses = dc.m_cross_num;
	num_Cars = dc.m_car_num;
	roads = dc.road;
	crosses = dc.cross;
	cars = dc.car;
	time_Scheduler = 0;//调度器初始时间设置为0
	vexnum = dc.getCrossNum();
	edge = dc.getRoadNum();
	tmp = dc.getArc(); //得到邻接矩阵
	tmp1 = dc.getRoadvArc(); //得到道路限速邻接矩阵
	//将graphC2R大小设置为36*36
	graphC2R = dc.graphC2R;
	num_CarsPut = 0;//已经发车的数量预置为0
	graphRoadStatusByDS = dc.graphRoadStatusByDS;
	speedType = dc.speedType;
	mapId2IndexCar = dc.mapId2IndexCar;
	mapId2IndexRoad = dc.mapId2IndexRoad;
	mapId2IndexCross = dc.mapId2IndexCross;

	for (int i = 0; i < graphRoadStatusByDS.size(); i++)
	{
		for (int j = 0; j < graphRoadStatusByDS[0].size(); j++)
		{
			graphRoadStatusByDS[i][j] = 0;
		}
	}

}

Scheduler::Scheduler(DataCenter * dc)
{
	num_CarsScheduling = dc->m_car_num;//获得需要调度的车数量
	num_Roads = dc->m_road_num;
	num_Crosses = dc->m_cross_num;
	num_Cars = dc->m_car_num;
	roads = dc->road;
	crosses = dc->cross;
	cars = dc->car;
	time_Scheduler = 0;//调度器初始时间设置为0
	vexnum = dc->getCrossNum();
	edge = dc->getRoadNum();
	tmp = dc->getArc(); //得到邻接矩阵
	tmp1 = dc->getRoadvArc(); //得到道路限速邻接矩阵
	//将graphC2R大小设置为36*36
	graphC2R = dc->graphC2R;
	num_CarsPut = 0;//已经发车的数量预置为0
	graphRoadStatusByDS = dc->graphRoadStatusByDS;
	speedType = dc->speedType;
	mapId2IndexCar = dc->mapId2IndexCar;
	mapId2IndexRoad = dc->mapId2IndexRoad;
	mapId2IndexCross = dc->mapId2IndexCross;

	for (int i = 0; i < graphRoadStatusByDS.size(); i++)
	{
		for (int j = 0; j < graphRoadStatusByDS[0].size(); j++)
		{
			graphRoadStatusByDS[i][j] = 0;
		}
	}
}

Scheduler::~Scheduler()
{
}

bool less_time(const Car & m1, const Car & m2) {
	return m1.time < m2.time;
}

bool more_location(const Car & m1, const Car & m2) {
	return m1.location > m2.location;
}


int Scheduler::getSysTime()
{
	SchedulerInit();
	while (num_CarsScheduling > 0)
	{
		SchedulerCore();
		driverCarInGarage();
		if (!putAllCarStatus())//输出所有车的状态
			return false;//发生死锁
		time_Scheduler++;//更新调度器时间
		putAllRoadStatus();
	}
	return time_Scheduler;
}

int Scheduler::getSysTimeV2()
{
	SchedulerInit();
	while (num_CarsScheduling > 0)
	{
		SchedulerCore_V2();
		driverCarInGarage();
		if (!putAllCarStatus())//输出所有车的状态
			return false;//发生死锁
		time_Scheduler++;//更新调度器时间
		putAllRoadStatus();
	}
	return time_Scheduler;
}

int Scheduler::SchedulerTest()
{
	int para = 80;
	getPath();

	for (int i = 0; i < 10; ++i)//迭代5次
	{
		ReOrderStartBySpeed(para);
		int time = getSysTime();
		int timeV2 = getSysTimeV2();
		para -= 3;
		PRINT("para:%d SysTime:%d   SysTimeV2:%d\n",para,time,timeV2);
	}

	/*
	ReOrderStartBySpeed(71);
	int time = getSysTime();
	int timeV2 = getSysTimeV2();
	PRINT("para:%d SysTime:%d   SysTimeV2:%d\n", para, time, timeV2);
	*/

	return 0;
}

int Scheduler::getSysTimeChangePath(int para)
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);
	SchedulerInit();
	while (num_CarsScheduling > 0)
	{
		SchedulerCore();
		driverCarInGarageDynamic(graph, para);
		if (!putAllCarStatus())//输出所有车的状态
			return false;//发生死锁
		putAllRoadStatus();
		time_Scheduler++;//更新调度器时间
	}
	return time_Scheduler;
}

int Scheduler::getSysTimeChangeTime(int para)
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	SchedulerInit();
	while (num_CarsScheduling > 0)
	{
		SchedulerCore();
		driverCarInGarageChangeTime(graph,para);
		if (!putAllCarStatus())//输出所有车的状态
			return false;//发生死锁
		putAllRoadStatus();
		time_Scheduler++;//更新调度器时间
	}
	return time_Scheduler;
}

void Scheduler::ReOrderStartByTime(int para)
{

	for (int j = 0; j < num_Cars; ++j)
	{
		if (cars[j].speed == 8 && cars[j].status == SLEEPING)
			carsInGarage.push_back(cars[j]);
	}
	//std::sort(carsInGarage.begin(), carsInGarage.end(), less_time);
	int size = carsInGarage.size();
	for (int i = 0; i < size; i++)
	{
		int index = i / (size /(2 * para));
		Car car = carsInGarage.front();//先调度时间长的
		carsInGarage.pop_front();
		if (index >= cars[car.index].plantime)
			cars[car.index].starttime = index;
		else
			cars[car.index].starttime = cars[car.index].plantime;
	}

	for (int j = 0; j < num_Cars; ++j)
	{
		if (cars[j].speed == 6 && cars[j].status == SLEEPING)
			carsInGarage.push_back(cars[j]);
	}
	//std::sort(carsInGarage.begin(), carsInGarage.end(), less_time);
	size = carsInGarage.size();
	for (int i = 0; i < size; i++)
	{
		int index = i / (size / (2 * para));
		Car car = carsInGarage.front();//先调度时间长的
		carsInGarage.pop_front();
		if (index >= cars[car.index].plantime)
			cars[car.index].starttime = 2 * para + index;
		else
			cars[car.index].starttime = 2 * para + cars[car.index].plantime;
	}

	for (int j = 0; j < num_Cars; ++j)
	{
		if (cars[j].speed == 4 && cars[j].status == SLEEPING)
			carsInGarage.push_back(cars[j]);
	}
	//std::sort(carsInGarage.begin(), carsInGarage.end(), less_time);
	size = carsInGarage.size();
	for (int i = 0; i < size; i++)
	{
		int index = i / (size / (2 * para));
		Car car = carsInGarage.front();//先调度时间长的
		carsInGarage.pop_front();
		if (index >= cars[car.index].plantime)
			cars[car.index].starttime = 4 * para + index;
		else
			cars[car.index].starttime = 4 * para + cars[car.index].plantime;
	}

	for (int j = 0; j < num_Cars; ++j)
	{
		if (cars[j].speed == 2 && cars[j].status == SLEEPING)
			carsInGarage.push_back(cars[j]);
	}
	//std::sort(carsInGarage.begin(), carsInGarage.end(), less_time);
	size = carsInGarage.size();
	for (int i = 0; i < size; i++)
	{
		int index = i / (size / (2 * para));
		Car car = carsInGarage.front();//先调度时间长的
		carsInGarage.pop_front();
		if (index >= cars[car.index].plantime)
			cars[car.index].starttime = 6 * para + index;
		else
			cars[car.index].starttime = 6 * para + cars[car.index].plantime;
	}
}

void Scheduler::ReOrderStartBySpeed(int para)
{
	int n2, n4, n6, n8;
	n2 = para;
	n4 = para;
	n6 = para - para/36;
	n8 = para - para/12;
	for (int i = 1; i <= num_Cars; ++i)//忽略第0行数据
	{
		switch (cars[i - 1].speed)
		{
		case 2:
			cars[i - 1].starttime = 2 * n8 + 2 * n6 + 2 * n4 + i % (2 * n2);
			break;
		case 4:
			cars[i - 1].starttime = 2 * n8 + 2 * n6 + i % (2 * n4);
			break;
		case 6:
			cars[i - 1].starttime = 2 * n8 + i % (2 * n6);
			break;
		case 8:
			cars[i - 1].starttime = 0 * n8 + i % (2 * n8);
			break;
		default:
			break;
		}

		cars[i - 1].idCurRoad = 0;
		cars[i - 1].idCurLane = 0;
		cars[i - 1].location = 0;//参数重置
		cars[i - 1].dirCross = NONE;//参数重置
		cars[i - 1].status= SLEEPING;//参数重置
		if (cars[i - 1].starttime < cars[i - 1].plantime)
		{
			cars[i - 1].starttime = cars[i - 1].plantime;
		}
		cars[i - 1].starttimeAnswer = cars[i - 1].starttime;//starttimeAnswer为最终写出的出发时间，不会更改
	}
}

bool Scheduler::addCarandChangeSTime(Car car)
{
	assert(car.status == SLEEPING);//只有SLEEPING状态的车可以加入地图行驶
	int idRoadTarget = car.path[0];//获取目标道路
	int idCrossTarget = car.idCrossFrom;//获得该车出发路口
	int idLaneStart = 0;
	int idLaneTarget = -1;//初始车道设置为-1，如果无车道可加入，则推迟
	int indexRoadTarget = id2indexRoad(idRoadTarget);
	if (idCrossTarget == roads[indexRoadTarget].idTo)
	{
		idLaneStart = roads[indexRoadTarget].channel;
	}
	for (int i = idLaneStart; i < idLaneStart + roads[indexRoadTarget].channel; ++i)
	{
		if (roads[indexRoadTarget].lane[i].laneCar.size() != 0)
		{
			if (roads[indexRoadTarget].lane[i].laneCar[roads[indexRoadTarget].lane[i].laneCar.size() - 1].location > 1)//留下一个空位
			{
				idLaneTarget = i;
				break;
			}
		}
		else
		{
			//如果该车道为空
			idLaneTarget = i;
			break;
		}
	}
	if (idLaneTarget == -1)
	{
		carsWaitInGarage.push_back(car);
		return false;
	}

	int locationTarget = 0;
	if (roads[indexRoadTarget].lane[idLaneTarget].laneCar.size() == 0)
	{
		locationTarget = std::min(car.speed, roads[indexRoadTarget].speed);
	}
	else
	{
		Car carNext = roads[indexRoadTarget].lane[idLaneTarget].laneCar[roads[indexRoadTarget].lane[idLaneTarget].laneCar.size() - 1];//目标车道的最后一辆车
		if (carNext.location > std::min(car.speed, roads[indexRoadTarget].speed))//不形成阻挡
		{
			locationTarget = std::min(car.speed, roads[indexRoadTarget].speed);
		}
		else
		{
			locationTarget = carNext.location - 1;
		}
	}
	car.status = FINESHED;//切换car的状态
	car.idCurRoad = idRoadTarget;
	car.idCurLane = idLaneTarget;
	car.location = locationTarget;
	car.dirCross = NONE;
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	roads[indexCarCurRoad].lane[car.idCurLane].laneCar.push_back(car);//将该车加入对应道路,对应车道,加入队尾

	num_CarsPut += 1;
	return true;
}

void Scheduler::driveAllCarsJustOnRoadToEndState()
{
	for (int i = 0; i < num_Roads; ++i)//按道路ID升序进行调度
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			if (roads[i].lane[j].laneCar.size() != 0)//先判断该车道是否有车
			{
				for (int m = 0; m < roads[i].lane[j].laneCar.size(); ++m)
				{
					//此阶段所有车都是终止状态
					driveCarStep1(roads[i].lane[j].laneCar[m], m);//从第一辆车开始往后调度
					assert(roads[i].lane[j].laneCar.size() != 0);
				}
			}
		}
	}
}

int Scheduler::driveCarNew(Car car)
{
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	int indexCar = roads[indexCarCurRoad].lane[car.idCurLane].laneCar.size() - 1;
	if (roads[indexCarCurRoad].lane[car.idCurLane].laneCar.size() == 1)//该车为该车道的第一辆车
	{
		roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].location += std::min(roads[indexCarCurRoad].speed, car.speed);//车正常行驶
		roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
		roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].dirCross = NONE;
	}
	else//该车前面有车
	{
		Car carNext = roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar - 1];
		if (car.location + std::min(roads[indexCarCurRoad].speed, car.speed) < carNext.location)
		{//前面的车不形成阻挡
			roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[indexCarCurRoad].speed, car.speed);//车正常行驶
			roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
			roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
		}
		else
		{//前面的车形成阻挡
			if (carNext.status == FINESHED)
			{
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].location = carNext.location - 1;//行驶到前车的后一个位置
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{
				return false;
			}
		}
	}
	return true;
}

Car Scheduler::getCarFromRoad(int idRoad, int dir)
{

	return Car();
}

void Scheduler::driveCarStep1(Car car, int indexCar)
{
	assert(car.status==FINESHED);
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	//判断此车会不会通过路口
	if (car.location + std::min(roads[indexCarCurRoad].speed, car.speed) <= roads[indexCarCurRoad].length)//不会驶出路口
	{
		if (indexCar == 0)//是第一辆车
		{
			roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[indexCarCurRoad].speed, car.speed);//车正常行驶
			roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
			roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
		}
		else//不是第一辆车
		{
			Car carNext = roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar - 1];
			if (car.location + std::min(roads[indexCarCurRoad].speed, car.speed) < carNext.location)
			{//前面的车不形成阻挡
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[indexCarCurRoad].speed, car.speed);//车正常行驶
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{//前面的车形成阻挡
				if (carNext.status == FINESHED)
				{
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].location = carNext.location - 1;//行驶到前车的后一个位置
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
				}
				else
				{
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//车标记为WAITTING
				}
			}
		}

	}
	else//将要驶出路口
	{
		if (indexCar == 0)//是第一辆车
		{
			//此车将行驶出路口，需要判断此车在路口的方向
			//判断车的方向
			int idNextCross = 0;
			if (car.idCurLane >= roads[indexCarCurRoad].channel)//逆向
				idNextCross = roads[indexCarCurRoad].idFrom;//此车即将驶入的路口
			else
				idNextCross = roads[indexCarCurRoad].idTo;//此车即将驶入的路口
			//根据假设AA，此时可能有车辆驶入终点
			if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
			{
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].status = WAITTING;
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].dirCross = DD;//从NONE改为DD，直行到达终点
			}
			else
			{
				int idNextRoad = car.path[0];//此车即将驶入的道路
				int idCurRoad = car.idCurRoad;//此车当前道路

				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//此车变为等待状态
			}
		}
		else//不是第一辆车
		{
			Car carNext = roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar - 1];
			if (carNext.status == FINESHED)
			{
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].location = carNext.location - 1;//行驶到前车的后一个位置
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{
				//此车也将行驶出路口，需要判断此车在路口的方向
				//判断车的方向
				int idNextCross = 0;
				if (car.idCurLane >= roads[indexCarCurRoad].channel)//逆向
					idNextCross = roads[indexCarCurRoad].idFrom;//此车即将驶入的路口
				else
					idNextCross = roads[indexCarCurRoad].idTo;//此车即将驶入的路口
				//根据假设AA，此时可能有车辆驶入终点
				if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
				{
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = DD;//从NONE改为DD，直行到达终点
				}
				else
				{
					int idNextRoad = car.path[0];//此车即将驶入的道路
					int idCurRoad = car.idCurRoad;//此车当前道路
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
					roads[indexCarCurRoad].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//此车变为等待状态
				}
			}
		}

	}
}

bool Scheduler::addCar(Car car, int i)
{
	assert(car.status == SLEEPING);//只有SLEEPING状态的车可以加入地图行驶
	int idRoadTarget = car.path[0];//获取目标道路
	int idCrossTarget = car.idCrossFrom;//获得该车出发路口
	int idLaneStart = 0;
	int idLaneTarget = -1;//初始车道设置为-1，如果无车道可加入，则推迟
	int indexCarRoadTarget = id2indexRoad(idRoadTarget);
	if (idCrossTarget == roads[indexCarRoadTarget].idTo)
	{
		idLaneStart = roads[indexCarRoadTarget].channel;
	}
	for (int i = idLaneStart; i < idLaneStart + roads[indexCarRoadTarget].channel; ++i)
	{
		if (roads[indexCarRoadTarget].lane[i].laneCar.size() != 0)
		{
			if (roads[indexCarRoadTarget].lane[i].laneCar[roads[indexCarRoadTarget].lane[i].laneCar.size() - 1].location > 1)
			{
				idLaneTarget = i;
				break;
			}
		}
		else
		{
			//如果该车道为空
			idLaneTarget = i;
			break;
		}
	}
	if (idLaneTarget == -1)
	{
		carsWaitInGarage.push_back(car);
		//cars[car.index].starttime += 1;
		return false;
	}

	int locationTarget = 0;
	if (roads[indexCarRoadTarget].lane[idLaneTarget].laneCar.size() == 0)
	{
		locationTarget = std::min(car.speed, roads[indexCarRoadTarget].speed);
	}
	else
	{
		Car carNext = roads[indexCarRoadTarget].lane[idLaneTarget].laneCar[roads[indexCarRoadTarget].lane[idLaneTarget].laneCar.size() - 1];//目标车道的最后一辆车
		if (carNext.location > std::min(car.speed, roads[indexCarRoadTarget].speed))//不形成阻挡
		{
			locationTarget = std::min(car.speed, roads[indexCarRoadTarget].speed);
		}
		else
		{
			locationTarget = carNext.location - 1;
		}
	}
	car.status = FINESHED;//切换car的状态
	car.idCurRoad = idRoadTarget;
	car.idCurLane = idLaneTarget;
	car.location = locationTarget;
	car.dirCross = NONE;
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	int indexCar = roads[indexCarCurRoad].lane[car.idCurLane].laneCar.size();//该车为末尾
	roads[indexCarCurRoad].lane[car.idCurLane].laneCar.push_back(car);//将该车加入对应道路,对应车道,加入队尾

	num_CarsPut += 1;
	return true;
}

int Scheduler::isCanEnter(int idRoad, int idCross)
{
	// ————————
	//     3
	//  <— — — —
	//     2
	//——————————
	//     0
	//  — — — —>
	//     1
	//——————————
	int indexRoad = id2indexRoad(idRoad);
	int idStartLane = 0;//如果cross为道路的入方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[indexRoad].idTo == crosses[idCross - 1].id)//如果cross为道路的入方向
	{
		idStartLane = roads[indexRoad].channel;
		assert(roads[indexRoad].isDuplex == 1);//如果cross为road的出方向，但是却不是双向道，那么很可能路径规划错误
	}
	for (int j = idStartLane; j < idStartLane + roads[indexRoad].channel; ++j)//遍历所有lane
	{
		if (roads[indexRoad].lane[j].laneCar.size() < roads[indexRoad].length)
		{
			//这里有问题，车的数量可能小于车道长度，但是最后一辆车有可能堵在最后一位
			if (roads[indexRoad].lane[j].laneCar.size() > 0)
			{
				Car carNext = roads[indexRoad].lane[j].laneCar[roads[indexRoad].lane[j].laneCar.size() - 1];
				if (carNext.location > 1)
				{
					return j;
				}
				else
				{
					if (carNext.status == WAITTING)
						return j;
				}
			}
			else
			{
				return j;//存在空位，可加入,返回车道id
			}
		}
		else
		{
			Car carNext = roads[indexRoad].lane[j].laneCar[roads[indexRoad].lane[j].laneCar.size() - 1];
			if (carNext.status == WAITTING)
				return j;
		}
	}
	return idStartLane + roads[indexRoad].channel - 1;//如果无空位返回最后一个车道
}

bool Scheduler::isBeDD(int idRoad, int idCross)//注意这里的ID不需要减1
{
	if (idRoad == -1)//如果冲突方向无道路，则任务无冲突车辆
		return false;
	int indexRoad = id2indexRoad(idRoad);
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[indexRoad].idFrom == crosses[idCross - 1].id)//如果cross为道路的出方向
	{
		idStartLane = roads[indexRoad].channel;
		if (roads[indexRoad].isDuplex != 1)
			return false;
	}
	//原本以为需要判断所有车道，现在只判断优先级最高的车道
	if (roads[indexRoad].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[indexRoad].lane[idStartLane].laneCar[0].dirCross == DD)
		{
			//只有等待状态的车会形成冲突
			if (roads[indexRoad].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//存在左转车辆
		}
	}
	return false;
}

bool Scheduler::isBeLEFT(int idRoad, int idCross)//注意这里的ID不需要减1
{
	if (idRoad == -1)//如果冲突方向无道路，则任务无冲突车辆
		return false;
	int indexRoad = id2indexRoad(idRoad);
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[indexRoad].idFrom == crosses[idCross - 1].id)//如果cross为道路的出方向
	{
		idStartLane = roads[indexRoad].channel;
		if (roads[indexRoad].isDuplex != 1)
			return false;
	}
	//原本以为需要判断所有车道，现在只判断优先级最高的车道
	if (roads[indexRoad].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[indexRoad].lane[idStartLane].laneCar[0].dirCross == LEFT)
		{
			//只有等待状态的车会形成冲突
			if (roads[indexRoad].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//存在左转车辆
		}
	}
	return false;
}

int Scheduler::getCrossDir(int idCurRoad, int idNextRoad, int idNextCross)
{
	int dirCurRoad = 0;//当前道路在路口的方向
	int dirNextRoad = 0;//即将驶入道路在路口的方向
	if (crosses[idNextCross - 1].roadID_T == idCurRoad)
		dirCurRoad = 0;
	else if (crosses[idNextCross - 1].roadID_R == idCurRoad)
		dirCurRoad = 1;
	else if (crosses[idNextCross - 1].roadID_D == idCurRoad)
		dirCurRoad = 2;
	else if (crosses[idNextCross - 1].roadID_L == idCurRoad)
		dirCurRoad = 3;

	if (crosses[idNextCross - 1].roadID_T == idNextRoad)
		dirNextRoad = 0;
	else if (crosses[idNextCross - 1].roadID_R == idNextRoad)
		dirNextRoad = 1;
	else if (crosses[idNextCross - 1].roadID_D == idNextRoad)
		dirNextRoad = 2;
	else if (crosses[idNextCross - 1].roadID_L == idNextRoad)
		dirNextRoad = 3;

	switch (dirNextRoad - dirCurRoad)
	{
	case 0:
		break;
	case 1:
		return LEFT;
		break;
	case -1:
		return RIGHT;
		break;
	case 2:
		return DD;
		break;
	case -2:
		return DD;
		break;
	case 3:
		return RIGHT;
		break;
	case -3:
		return LEFT;
		break;
	default:
		break;
	}
}

int Scheduler::getCrossDistance(Car car, int idCurRoad, int idNextRoad)
{
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	int indexNextRoad = id2indexRoad(idNextRoad);
	int disCurRoad = roads[indexCarCurRoad].length - car.location;//当前道路可以行驶的距离
	int disNextRoadMax = std::min(roads[indexNextRoad].speed, car.speed);//下一道路可以行驶的最大距离
	assert(disCurRoad >= 0);
	assert(disNextRoadMax >= 0);
	if (disCurRoad >= disNextRoadMax)
		return 0;
	else
		return disNextRoadMax - disCurRoad;
}

void Scheduler::driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location)
{
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	int indexCarNextRoad = id2indexRoad(idNextRoad);
	assert(location > 0);
	std::vector<Car>::iterator it = roads[indexCarCurRoad].lane[car.idCurLane].laneCar.begin();
	roads[indexCarCurRoad].lane[car.idCurLane].laneCar.erase(it);	//将该车从当前lane删除,该车肯定是当前lane的第一辆车
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//该车路口状态设置为NONE,代表已经驶离路口
	car.status = FINESHED;//该车调度完成，等待下一时间片再行驶
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	roads[indexCarNextRoad].lane[car.idCurLane].laneCar.push_back(car);//将该车加入下一个lane,加入队尾
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	int indexCarCurRoad = id2indexRoad(car.idCurRoad);
	assert(crosses[idCross - 1].roadID[dir] != -1);//不可能进入无效路径
	int idNextCross = 0;
	if (car.idCurLane >= roads[indexCarCurRoad].channel)//逆向
		idNextCross = roads[indexCarCurRoad].idFrom;//此车即将驶入的路口
	else
		idNextCross = roads[indexCarCurRoad].idTo;//此车即将驶入的路口
	int idNextRoad = car.path[0];//获取目标道路
	int indexNextRoad = id2indexRoad(idNextRoad);
	int idNextLane = isCanEnter(idNextRoad, idNextCross);

	int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
	if (disNextRoad == 0)//可行驶距离为0，则停在当前路口
	{
		roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].location = roads[indexCarCurRoad].length;
		roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
		roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].dirCross = NONE;
		return true;
	}
	else
	{
		int sizeLaneCar = roads[indexNextRoad].lane[idNextLane].laneCar.size();
		if (sizeLaneCar > 0)
		{
			Car carNext = roads[indexNextRoad].lane[idNextLane].laneCar[sizeLaneCar - 1];
			if (disNextRoad < carNext.location)
			{
				driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);
				return true;
			}
			else
			{
				if (carNext.status == WAITTING)
				{
					return false;
				}
				else
				{
					if (carNext.location > 1)
					{
						driverToNextRoad(car, idNextRoad, idNextLane, carNext.location - 1);
						return true;
					}
					else
					{
						roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].location = roads[indexCarCurRoad].length;
						roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
						roads[indexCarCurRoad].lane[car.idCurLane].laneCar[0].dirCross = NONE;
						return true;
					}
						
				}
			}
		}
		else
		{
			driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);
			return true;
		}
	}
}

void Scheduler::driverCarInGarage()
{
	
	int numCarsWait = carsWaitInGarage.size();
	for (int j = 0; j < numCarsWait; ++j)
	{
		Car car = carsWaitInGarage.front();
		carsWaitInGarage.pop_front();
		addCar(car, car.index);
	}
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].starttime == time_Scheduler && cars[i].status == SLEEPING)
		{
			addCar(cars[i], i);
			//addCarandChangeSTime(cars[i]);
		}
	}
}

void Scheduler::driverCarInGarageDynamic(Graph_DG &graph,int para)
{
	int numCarsWait = carsWaitInGarage.size();
	for (int j = 0; j < numCarsWait; ++j)
	{
		Car car = carsWaitInGarage.front();
		carsWaitInGarage.pop_front();
		addCar(car, car.index);
	}
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].starttime == time_Scheduler && cars[i].status == SLEEPING)
		{
			int timeCar = 0;
			if (cars[i].speed == 8) para = 7;
			else if (cars[i].speed == 6) para = 7;
			else if (cars[i].speed == 4) para = 7;
			else if (cars[i].speed == 2) para = 7;

			vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed, graphRoadStatusByDS, para, timeCar);
			cars[i].time = timeCar;
			//cross矩阵转road矩阵
			vector<int> pathRoad(pathCross.size() - 1);
			for (int j = 0; j < pathRoad.size(); ++j)
			{
				pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			}
			cars[i].path = pathRoad;
			addCar(cars[i], i);
		}
	}
}

void Scheduler::driverCarInGarageChangeTime(Graph_DG &graph,int para)
{
	int timeCar = 0;

	static vector<Car> reorderCar;
	static int flag = 0;
	//对cars按照车辆的速度类型进行排序，存放到qCar中
	if (flag == 0)//因为是静态变量所以只需要重排序一次就可以了
	{
		reorderCars(reorderCar);
		reverse(reorderCar.begin(), reorderCar.end());//将qCar的速度从大到小重排序一次
	}

	std::vector<Car> carsDeparture;//此时刻待发车辆
	//从所有即将出发的车中挑选车来出发
	float loadAdded = 0;
	int carnum = 0;
	for (int i = 0; i < num_Cars; ++i)
	{
		if (time_Scheduler > 0 && reorderCar[i].starttime == 0)
		{
			vector<int> pathCross = graph.Dijkstra(reorderCar[i].idCrossFrom, reorderCar[i].idCrossTo, reorderCar[i].speed, timeCar);
			vector<int> pathRoad(pathCross.size() - 1);
			vector<int> pathRoadWithDir(pathCross.size() - 1);
			for (int j = 0; j < pathRoad.size(); ++j)
			{
				pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
				if (roads[pathRoad[j] - 5000].idFrom == pathCross[j])
					pathRoadWithDir[j] = pathRoad[j];
				else
					pathRoadWithDir[j] = -pathRoad[j];
			}
			reorderCar[i].path = pathRoad;
			reorderCar[i].time = timeCar;
			cars[reorderCar[i].id - 10000].path = pathRoad;
			cars[reorderCar[i].id - 10000].time = timeCar;

			if (reorderCar[i].speed == 8)
				loadAdded = loadAdded + timeCar;
			if (reorderCar[i].speed == 6)
				loadAdded = loadAdded + timeCar * 0.7;
			if (reorderCar[i].speed == 4)
				loadAdded = loadAdded + timeCar * 0.4;
			if (reorderCar[i].speed == 2)
				loadAdded = loadAdded + timeCar * 0.1;

			//loadAdded += timeCar;
			carnum++;
			if (loadAdded > 280 || carnum > 35)
				break;

			//计算该车能否此次加入系统
			if (judgement(mapForJamDegree, pathRoadWithDir))
			{
				reorderCar[i].starttime = time_Scheduler;
				cars[reorderCar[i].id - 10000].starttime = time_Scheduler;
				reorderCar[i].starttimeAnswer = reorderCar[i].starttime;
				cars[reorderCar[i].id - 10000].starttimeAnswer = cars[reorderCar[i].id - 10000].starttime;
				carsDeparture.push_back(reorderCar[i]);
			}
		}
	}
	int numCarsWait = carsWaitInGarage.size();
	for (int j = 0; j < numCarsWait; ++j)
	{
		Car car = carsWaitInGarage.front();
		carsWaitInGarage.pop_front();
		addCar(car, car.id - 10000);
	}
	for (auto car : carsDeparture)
	{
		addCar(car, car.id - 10000);
	}
	carsDeparture.clear();
	flag++;
}

void Scheduler::putCarStatus(Car car)
{
	if(car.path.size()!=0)
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  NextRoad:%d  Status:%d  Dir:%d\n",
		car.id,car.idCrossFrom,car.idCrossTo,car.idCurRoad,car.idCurLane,car.location,car.path[0], car.status, car.dirCross);//鎵撳嵃绯荤粺鏃堕棿
	else
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  Status:%d  Dir:%d\n",
			car.id, car.idCrossFrom, car.idCrossTo, car.idCurRoad, car.idCurLane, car.location, car.status, car.dirCross);//鎵撳嵃绯荤粺鏃堕棿
}

bool Scheduler::putAllCarStatus()
{
	bool isDeadLock = false;
	vector<int> vecDLCross;
	for (int i = 0; i < num_Roads; ++i)//按道路ID升序进行调度
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			Lane lane = roads[i].lane[j];

			if (lane.laneCar.size() != 0)//先判断该车道是否有车
			{
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					if (lane.laneCar[m].status == WAITTING)
					{
						int idDLCross = 0;
						carsDeadLock.push_back(lane.laneCar[m]);//将死锁的车加入队列
						if (j > roads[i].channel)//行驶在反向车道上
						{
							idDLCross = roads[i].idTo;
						}
						else
						{
							idDLCross = roads[i].idFrom;
						}
						//putCarStatus(lane.laneCar[m]);
						vector<int>::iterator it;
						it = find(vecDLCross.begin(), vecDLCross.end(), idDLCross);
						if (it == vecDLCross.end())
						{
							vecDLCross.push_back(idDLCross);
						}
						isDeadLock = true;
					}
				}
			}
		}
	}
	if (isDeadLock == true)
	{
		PRINT("Dead Time:%d  Dead Lock Cross ID:",time_Scheduler);
		for (auto idCross : vecDLCross)
		{
			PRINT("%d ",idCross);
		}
		PRINT("\n");
	}
	return !isDeadLock;
}

void Scheduler::putAllRoadStatus()
{
	int num_road_jam = 0;//堵住的道路数量，per高于0.7则认为堵车
	int num_cars_road = 0;
	for (int i = 0; i < num_Roads; ++i)
	{
		float perRoad = 0;
		float threshold = 0.5;
		if (roads[i].isDuplex)
		{
			for (int j = 0; j < roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
				num_cars_road += roads[i].lane[j].laneCar.size();
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idFrom - 1][roads[i].idTo - 1] = perRoad;//更新拥堵情况矩阵

			mapUpdate(mapForJamDegree, roads[i].id, perRoad);
			perRoad = 0;
			for (int j = roads[i].channel; j < 2 * roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
				num_cars_road += roads[i].lane[j].laneCar.size();
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idTo - 1][roads[i].idFrom - 1] = perRoad;//更新拥堵情况矩阵

			mapUpdate(mapForJamDegree, -roads[i].id, perRoad);//道路反方向的路况以id的负值来存储
			perRoad = 0;
		}
		else
		{
			for (int j = 0; j < roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
				num_cars_road += roads[i].lane[j].laneCar.size();
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idFrom - 1][roads[i].idTo - 1] = perRoad;//更新拥堵情况矩阵
			mapUpdate(mapForJamDegree, roads[i].id, perRoad);

		}
	}
	vec_numCarsInRoadPerTime.push_back(num_cars_road);
}

int Scheduler::getFirstRoadFromCross(int idCross, int index)
{
	std::vector<int> idRoad;
	for (int i = 0; i < 4; ++i)
	{
		if (crosses[idCross - 1].roadID[i] == -1)
			idRoad.push_back(INT_MAX);
		else
			idRoad.push_back(crosses[idCross - 1].roadID[i]);
	}
	std::sort(idRoad.begin(),idRoad.end());
	for (int i = 0; i < 4; ++i)
	{
		if (idRoad[i] == INT_MAX)
			idRoad[i] = -1;
	}
	return idRoad[index];
}

int Scheduler::getDirByRoadCrossDir(int idCross, int idRoad)
{
	int dirFrom = -1;//设置初值为-1，便于后续错误检测
	for (int i = 0; i < 4; ++i)
	{
		if (crosses[idCross - 1].roadID[i] == idRoad)
			dirFrom = i;
	}
	assert(dirFrom != -1);//如果未在该cross找到该road，则报错

	return dirFrom;
}

void Scheduler::driveAllCarsJustOnOneChannelToEndState(int idRoad, int idCross, int idChannel)
{
	int indexRoad = id2indexRoad(idRoad);
	if (roads[indexRoad].lane[idChannel].laneCar.size() != 0)
	{
		for (int i = 0; i < roads[indexRoad].lane[idChannel].laneCar.size(); ++i)
		{
			if (roads[indexRoad].lane[idChannel].laneCar[i].status == WAITTING)//只处理等待状态的车
			{
				Car car = roads[indexRoad].lane[idChannel].laneCar[i];
				if (car.location + std::min(roads[indexRoad].speed, car.speed) <= roads[indexRoad].length)//不会驶出路口
				{
					//只处理行驶后不通过路口的车
					if (i != 0)//如果该车不是第一辆车
					{
						Car carNext = roads[indexRoad].lane[idChannel].laneCar[i - 1];
						if (car.location + std::min(roads[indexRoad].speed, car.speed) < carNext.location)
						{
							//前车不形成阻挡
							roads[indexRoad].lane[idChannel].laneCar[i].location += std::min(roads[indexRoad].speed, car.speed);//车正常行驶
							roads[indexRoad].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
							roads[indexRoad].lane[idChannel].laneCar[i].dirCross = NONE;
						}
						else if (carNext.status == FINESHED)
						{
							roads[indexRoad].lane[idChannel].laneCar[i].location = carNext.location - 1;//车正常行驶
							roads[indexRoad].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
							roads[indexRoad].lane[idChannel].laneCar[i].dirCross = NONE;
						}
					}
					else
					{
						//前车不形成阻挡
						roads[indexRoad].lane[idChannel].laneCar[i].location += std::min(roads[indexRoad].speed, car.speed);//车正常行驶
						roads[indexRoad].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
						roads[indexRoad].lane[idChannel].laneCar[i].dirCross = NONE;
					}
				}
				else 
				{
					//只处理行驶后不通过路口的车
					if (i != 0)//如果该车不是第一辆车
					{
						Car carNext = roads[indexRoad].lane[idChannel].laneCar[i - 1];
						if (carNext.status == FINESHED)
						{
							roads[indexRoad].lane[idChannel].laneCar[i].location = carNext.location - 1;//车正常行驶
							roads[indexRoad].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
							roads[indexRoad].lane[idChannel].laneCar[i].dirCross = NONE;
						}
					}
					else
					{
						carsRoadWaitting.push_back(car);
						std::sort(carsRoadWaitting.begin(), carsRoadWaitting.end(), more_location);
					}
				}
			}
		}
	}
}

void Scheduler::getPlantimeByPeriod(int period)
{
	for (int i = 0; i < num_Cars; ++i)//忽略第0行数据
	{
		int n2, n4, n6, n8;
		n2 = period;
		n4 = period;
		n6 = period;
		n8 = period;
		switch (cars[i].speed)
		{
		case 2:
			cars[i].starttime = cars[i].plantime + 6 * n2 + i % (2 * n2 - 10);
			break;
		case 4:
			cars[i].starttime = cars[i].plantime + 4 * n4 + i % (2 * n4 - 10);
			break;
		case 6:
			cars[i].starttime = cars[i].plantime + 2 * n6 + i % (2 * n6 - 10);
			break;
		case 8:
			cars[i].starttime = cars[i].plantime + 0 * n8 + i % (2 * n8 - 10);
			break;
		default:
			break;
		}
		cars[i].status = SLEEPING;
		cars[i].dirCross = NONE;
		cars[i].location = 0;
		cars[i].idCurRoad = 0;
		cars[i].idCurLane = 0;
	}
}

int Scheduler::id2indexCar(int id)
{
	map<int, int>::iterator it;
	it = mapId2IndexCar.find(id);
	assert(it!= mapId2IndexCar.end());
	return it->second;
}

int Scheduler::id2indexRoad(int id)
{
	map<int, int>::iterator it;
	it = mapId2IndexRoad.find(id);
	assert(it != mapId2IndexRoad.end());
	return it->second;
}

int Scheduler::id2indexCross(int id)
{
	map<int, int>::iterator it;
	it = mapId2IndexCross.find(id);
	assert(it != mapId2IndexCross.end());
	return it->second;
}

int Scheduler::SchedulerInit()
{
	time_Scheduler = 0;//调度器时间归零
	num_CarsScheduling = num_Cars;//调度车的数量重新赋值
	carsWaitInGarage.clear();//等待车库归零
	carsDeadLock.clear();//死锁车归零
	vec_numCarsInRoadPerTime.clear();//每个时刻道路中车数量归零
	carsRoadWaitting.clear();
	for (int i = 0; i < num_Roads; ++i)
	{
		int idLaneStart = 0;
		if (roads[i].isDuplex)
		{
			idLaneStart = roads[i].channel;
		}
		for (int j = 0; j < idLaneStart + roads[i].channel; ++j)
		{
			roads[i].lane[j].laneCar.clear();
		}
	}//道路中车辆归零
	for (int i = 0; i < num_Cars; ++i)//忽略第0行数据
	{
		cars[i].idCurRoad = 0;//参数重置
		cars[i].idCurLane = 0;//参数重置
		cars[i].location = 0;//参数重置
		cars[i].dirCross = NONE;//参数重置
		cars[i].status = SLEEPING;//参数重置
	}//车辆信息归零
	return 0;
}

int Scheduler::SchedulerCore()
{
	while (num_CarsScheduling > 0)
	{
		/*第一步：先处理所有道路上的车辆，进行遍历扫描*/
		driveAllCarsJustOnRoadToEndState();

		/*第二步：先处理所有道路上的车辆，进行遍历扫描*/
		while (1)//终止条件为：一个循环后，没有任何车被调度
		{
			bool isWorkingCross = false;//标志变量，如果一个循环后没有任何一辆车被调度，则退出循环
			for (int i = 0; i < num_Crosses; ++i)////按照升序调度所有路口
			{
				int idCross = crosses[i].id;//获得路口ID
				int indexCross = i;
				//while (1)//循环调度路口四个方向的车，直到全部车辆完成调度，或者阻塞
				//{
				bool isWorkingRoad = false;
				bool isConflict = false;
				for (int j = 0; j < 4; ++j)//这里按要求是根据道路id进行升序调度
				{
				CONFLICT:
					if (isConflict)
					{
						isConflict = false;
						j++;
					}
					if (j >= 4)
						break;
					int idRoad = getFirstRoadFromCross(idCross, j);
					if (idRoad != -1)
					{
						carsRoadWaitting.clear();
						int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
						if (roads[idRoad - 5000].idFrom == crosses[i].id)//如果cross为道路的入方向
						{
							idStartLane = roads[idRoad - 5000].channel;
							if (roads[idRoad - 5000].isDuplex != 1)
								continue;//如果非双车道，退出本次循环
						}
						while (1)
						{
							bool isWorkingLane = false;
							for (int m = idStartLane; m < idStartLane + roads[idRoad - 5000].channel; ++m)//遍历所有lane
							{
								if (roads[idRoad - 5000].lane[m].laneCar.size() != 0)
								{
									Car car = roads[idRoad - 5000].lane[m].laneCar[0];
									if (car.status == WAITTING)//只处理在路口且为等待状态的车
									{
										assert(car.status == WAITTING);//车辆在路口调度时一定要是WAITTING状态
										int dirConflict = 0;
										int dirTarget = 0;
										int idNextCross = 0;
										std::vector<Car>::iterator itCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
										switch (car.dirCross)
										{
										case NONE:
											PRINT("WARNNING!!!\n");
											break;
										case DD://直行>左转>右转
											//根据官方说明，即将到达终点的车以直行方式进入路口
											/***************************************************************/
											if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
												idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
											else
												idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
											//根据假设AA，此时可能有车辆驶入终点
											if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
											{
												num_CarsScheduling -= 1;//正在调度的车辆数减一
												roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(itCar);//删除该道路第一辆车
												isWorkingCross = true;
												isWorkingRoad = true;
												isWorkingLane = true;
												driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
												//记录到达时间
												cars[car.id - 10000].timeArrived = time_Scheduler;

												break;
												//该车准备通过路口
											}
											/***************************************************************/
											dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 2;//目标方向
											if (dirTarget > 3) dirTarget -= 4;//修正方向
											if (isCanDriveToNextRoad(car, dirTarget, idCross))
											{
												isWorkingCross = true;
												isWorkingRoad = true;
												isWorkingLane = true;
												driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
											}
											else
											{
												isConflict = true;
												goto CONFLICT;
											}

											//判断转入的road是否可以行驶

											break;
										case LEFT://左转>右转
											//判断即将转入的方向是否有直行进入的车辆
											dirConflict = getDirByRoadCrossDir(idCross, idRoad) - 1;//冲突方向
											if (dirConflict < 0) dirConflict += 4;//修正方向
											if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
											{
												dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 1;//目标方向
												if (dirTarget > 3) dirTarget -= 4;//修正方向
												if (isCanDriveToNextRoad(car, dirTarget, idCross))//判断转入的road是否可以行驶
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
													driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
												}
												else
												{
													isConflict = true;
													goto CONFLICT;
												}
											}
											else
											{
												isConflict = true;
												goto CONFLICT;
											}
											break;
										case RIGHT://右转优先级最低
											//判断即将转入的方向是否有直行进入的车辆
											dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 1;//冲突方向
											if (dirConflict > 3) dirConflict -= 4;//修正方向
											if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
											{
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 2;//冲突方向
												if (dirConflict > 3) dirConflict -= 4;//修正方向
												//判断即将转入的方向是否有左转进入的车辆
												if (!isBeLEFT(crosses[i].roadID[dirConflict], idCross))
												{
													dirTarget = getDirByRoadCrossDir(idCross, idRoad) - 1;//目标方向
													if (dirTarget < 0) dirTarget += 4;//修正方向
													if (isCanDriveToNextRoad(car, dirTarget, idCross))//判断转入的road是否可以行驶
													{
														isWorkingCross = true;
														isWorkingRoad = true;
														isWorkingLane = true;
														driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, m);
													}
													else
													{
														isConflict = true;
														goto CONFLICT;
													}
												}
												else
												{
													isConflict = true;
													goto CONFLICT;
												}
											}
											else
											{
												isConflict = true;
												goto CONFLICT;
											}
											break;
										default:
											PRINT("WARNNING!!!\n");
											break;
										}
									}
								}
							}
							if (!isWorkingLane)
								break;
						}
					}
				}
			}
			if (!isWorkingCross)//如果一个循环后没有任何一辆车被调度，则退出调度循环
				break;
		}
		return 0;
	}
}

int Scheduler::SchedulerCore_V2()
{
	while (num_CarsScheduling > 0)
	{
		/*第一步：先处理所有道路上的车辆，进行遍历扫描*/
		driveAllCarsJustOnRoadToEndState();

		/*第二步：先处理所有道路上的车辆，进行遍历扫描*/
		while (1)//终止条件为：一个循环后，没有任何车被调度
		{
			bool isDeadLock = true;
			for (int i = 0; i < num_Crosses; ++i)////按照升序调度所有路口
			{
				int idCross = crosses[i].id;//获得路口ID
				for (int j = 0; j < 4; ++j)//这里按要求是根据道路id进行升序调度
				{
					bool isConflict = false;
					int idRoad = getFirstRoadFromCross(idCross, j);
					if (idRoad != -1)
					{
						carsRoadWaitting.clear();
						int indexRoad = id2indexRoad(idRoad);
						int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
						if (roads[indexRoad].idFrom == crosses[i].id)//如果cross为道路的入方向
						{
							idStartLane = roads[indexRoad].channel;
							if (roads[indexRoad].isDuplex != 1)
								continue;//如果非双车道，退出本次循环
						}
						//在这里存入需要调度的车
						std::vector<Car> vec_carsPerRoad;
						std::vector<Car> vec_carsPerLine;
						for (int m = idStartLane; m < idStartLane + roads[indexRoad].channel; ++m)//遍历所有lane
						{
							if (roads[indexRoad].lane[m].laneCar.size() > 0)
							{
								if (roads[indexRoad].lane[m].laneCar[0].status == WAITTING
									&& roads[indexRoad].lane[m].laneCar[0].dirCross != NONE)
								{
									carsRoadWaitting.push_back(roads[indexRoad].lane[m].laneCar[0]);
								}
							}
						}
						std::sort(carsRoadWaitting.begin(), carsRoadWaitting.end(), more_location);
						
						for (int n = roads[indexRoad].length; n > 0; n--)
						{
							for (int m = idStartLane; m < idStartLane + roads[indexRoad].channel; ++m)//遍历所有lane
							{
								for (int t = 0; t < roads[indexRoad].lane[m].laneCar.size(); t++)
								{
									if (roads[indexRoad].lane[m].laneCar[t].location == n
										&& roads[indexRoad].lane[m].laneCar[t].status == WAITTING
										&& roads[indexRoad].lane[m].laneCar[t].dirCross != NONE)
									{
										vec_carsPerLine.push_back(roads[indexRoad].lane[m].laneCar[t]);
									}
								}
							}
							//std::sort(vec_carsPerLine.begin(), vec_carsPerLine.end(), more_location);
							for (auto car : vec_carsPerLine)
							{
								vec_carsPerRoad.push_back(car);
							}
							vec_carsPerLine.clear();
						}
						while(carsRoadWaitting.size() > 0)
						//for (auto car : vec_carsPerRoad)
						{
							Car car = carsRoadWaitting.front();
							carsRoadWaitting.pop_front();
							
							assert(car.status == WAITTING);
							int dirConflict = 0;
							int dirTarget = 0;
							int idNextCross = 0;
							int indexCarCurRoad = id2indexRoad(car.idCurRoad);
							int indexCar = id2indexCar(car.id);
							std::vector<Car>::iterator itCar = roads[indexCarCurRoad].lane[car.idCurLane].laneCar.begin();
							switch (car.dirCross)
							{
							case DD://直行>左转>右转
								//根据官方说明，即将到达终点的车以直行方式进入路口
								if (car.idCurLane >= roads[indexCarCurRoad].channel)//逆向
									idNextCross = roads[indexCarCurRoad].idFrom;//此车即将驶入的路口
								else
									idNextCross = roads[indexCarCurRoad].idTo;//此车即将驶入的路口
								//根据假设AA，此时可能有车辆驶入终点
								if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
								{
									num_CarsScheduling -= 1;//正在调度的车辆数减一
									roads[indexCarCurRoad].lane[car.idCurLane].laneCar.erase(itCar);//删除该道路第一辆车
									driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, car.idCurLane);
									//记录到达时间
									cars[indexCar].timeArrived = time_Scheduler;

									break;
									//该车准备通过路口
								}
								dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 2;//目标方向
								if (dirTarget > 3) dirTarget -= 4;//修正方向
								if (isCanDriveToNextRoad(car, dirTarget, idCross))
								{
									driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, car.idCurLane);
									isDeadLock = false;
								}
								else
								{
									isConflict = true;
								}
								break;
							case LEFT://左转>右转
								//判断即将转入的方向是否有直行进入的车辆
								dirConflict = getDirByRoadCrossDir(idCross, idRoad) - 1;//冲突方向
								if (dirConflict < 0) dirConflict += 4;//修正方向
								if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
								{
									dirTarget = getDirByRoadCrossDir(idCross, idRoad) + 1;//目标方向
									if (dirTarget > 3) dirTarget -= 4;//修正方向
									if (isCanDriveToNextRoad(car, dirTarget, idCross))//判断转入的road是否可以行驶
									{
										driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, car.idCurLane);
										isDeadLock = false;
									}
									else
									{
										isConflict = true;
									}
								}
								else
								{
									isConflict = true;
								}
								break;
							case RIGHT://右转优先级最低
								//判断即将转入的方向是否有直行进入的车辆
								dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 1;//冲突方向
								if (dirConflict > 3) dirConflict -= 4;//修正方向
								if (!isBeDD(crosses[i].roadID[dirConflict], idCross))
								{
									dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 2;//冲突方向
									if (dirConflict > 3) dirConflict -= 4;//修正方向
									//判断即将转入的方向是否有左转进入的车辆
									if (!isBeLEFT(crosses[i].roadID[dirConflict], idCross))
									{
										dirTarget = getDirByRoadCrossDir(idCross, idRoad) - 1;//目标方向
										if (dirTarget < 0) dirTarget += 4;//修正方向
										if (isCanDriveToNextRoad(car, dirTarget, idCross))//判断转入的road是否可以行驶
										{
											isDeadLock = false;
											driveAllCarsJustOnOneChannelToEndState(idRoad, idCross, car.idCurLane);
										}
										else
										{
											isConflict = true;
										}
									}
									else
									{
										isConflict = true;
									}
								}
								else
								{
									isConflict = true;
								}
								break;
							default:
								PRINT("WARNNING!!!\n");
								break;
							}
							if (isConflict == true)
							{
								break;
							}
						}
						vec_carsPerRoad.clear();
					}
				}
			}
			bool isAllFinished = true;

			for (int i = 0; i < num_Roads; ++i)//按道路ID升序进行调度
			{
				for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
				{
					Lane lane = roads[i].lane[j];

					if (lane.laneCar.size() != 0)//先判断该车道是否有车
					{
						for (int m = 0; m < lane.laneCar.size(); ++m)
						{
							if (lane.laneCar[m].status == WAITTING)
							{
								isAllFinished = false;
								break;
							}
						}
					}
				}
				if (isAllFinished == false)
					break;
			}
			if (isAllFinished == true)
				break;

			if (isDeadLock == true)
				break;
		}
		return 0;
	}
}

int Scheduler::unlockDead(int para)
{
	std::map<int, int> mapResult;
	int timeMax = INT_MAX;
	int time, timeFinal;
	int w = 9;
	for (int i = 0; i < 15; ++i)//迭代20次
	{
		ReOrderStartBySpeed(para);
		getPath();
		int time = getSysTimeChangePath(w);
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
	ReOrderStartBySpeed(para);
	getPath();
	time = getSysTimeChangePath(w);
	//修改死锁车的出发时间
	for (int i = 0; i < carsDeadLock.size() / 2; ++i)
	{
		Car car = carsDeadLock[i];
		cars[car.index].starttime += car.id % 60;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock1:%d\n", time);
	for (int i = 0; i < carsDeadLock.size() / 2; ++i)
	{
		Car car = carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock2:%d\n", time);
	for (int i = 0; i < carsDeadLock.size() / 2; ++i)
	{
		Car car = carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock3:%d\n", time);
	for (int i = 0; i < carsDeadLock.size() / 2; ++i)
	{
		Car car = carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock4:%d\n", time);
	for (int i = 0; i < carsDeadLock.size() / 2; ++i)
	{
		Car car = carsDeadLock[i];
		cars[car.index].starttime += car.id % 100;//出发时间重安排
		cars[car.index].starttimeAnswer = cars[car.index].starttime;
	}
	time = getSysTimeChangePath(w);//重新跑一下看是不是死锁
	PRINT("timeUnlock5:%d\n", time);
	time = getSysTime();
	PRINT("timeFinal:%d\n", time);
	/*
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (timeFinal - 20))
		{
			cars[i].starttime = cars[i].starttime - 20;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	time = getSysTime();
	PRINT("timeFinal:%d\n", time);
	*/

	carsDeadLock.clear();//清空死锁队列
	return 0;
}

void Scheduler::getPath()//获得最短路径和该路径下的运行时间
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	
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

void Scheduler::getPathWeightOne()
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);

	for (int i = 0; i < num_Cars; ++i)
	{
		int timeCar = 0;
		vector<int> pathCross = graph.DijkstraWeightOne(cars[i].idCrossFrom, cars[i].idCrossTo, timeCar);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
		cars[i].time = timeCar;
	}
}

bool Comp(const int &a, const int &b)
{
	return a > b;
}

void Scheduler::getStartTime(int para)
{
	//car_speed_num 为车辆速度类型数量 speedType存放速度类型的vector
	//para参数为每个时间片可发运行时间总和
	//car[0].time为每辆车运行时间

	std::deque<Car> carsDeque;//此时间片待出发的车 //deque是分配在堆中的，所以此处临时deque不会造成栈溢出
	int timeStart = 1;//出发时间，从1开始安排

	sort(speedType.begin(), speedType.end(), Comp);
	for (auto speed : speedType)
	{
		assert(carsDeque.size()==0);//使用前确保deque为空
		for (int i = 0; i < num_Cars; ++i)
		{
			if (cars[i].speed == speed)
			{
				carsDeque.push_back(cars[i]);
			}
		}
		std::sort(carsDeque.begin(), carsDeque.end(), less_time);//将deque中的车按照运行时间升序排列
		//当然如果不排序也是有一定道理的
		int loadCur;//记录当前负载
		while (carsDeque.size() > 0)
		{
			loadCur = 0;
			while (loadCur < para)
			{
				if (carsDeque.size() == 1)
				{
					Car car = carsDeque.front();
					carsDeque.pop_front();
					cars[car.index].starttime = timeStart;
					break;
				}
				else if (carsDeque.size() == 0)
				{
					break;
				}
				Car carShortTime = carsDeque.front();
				Car carLongTime = carsDeque.back();
				carsDeque.pop_front();
				carsDeque.pop_back();
				loadCur += carLongTime.time + carShortTime.time;
				cars[carLongTime.index].starttime = timeStart;
				cars[carShortTime.index].starttime = timeStart;
			}
			timeStart++;
		}
	}
	//防止starttime早于计划时间
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].starttime < cars[i].plantime)
		{
			cars[i].starttime = cars[i].plantime;
		}
	}
}

void Scheduler::getStartTime_loadbalance(int carnum)
{
	//car_speed_num 为车辆速度类型数量 speedType存放速度类型的vector
	//para参数为每个时间片可发运行时间总和
	//car[0].time为每辆车运行时间

	//carnum为同一时刻道路上的行驶的车辆总数
	//balance是一个二维vector用来记录当前道路上的车辆情况，当值为ture的时候表明有车，false没有
	// |	|	|	|
	// |	|	|	|
	// |	|	|	|
	static vector<vector<bool> > balance(carnum, vector<bool>(1, false));

	int timeStart = 1;//出发时间，从1开始安排
	
	while (qCar.size() > 0)
	{
		Car tmp;
		//遍历当前道路上的车辆情况
		for (int i = 0; i < carnum; i++)
		{
			if (balance[i][timeStart - 1] == false)//当前没有车辆，将车辆的行驶时间添加进去
			{

				if (qCar.size() == 0)
					goto L1;
				tmp = qCar.front();
				if (tmp.plantime > timeStart)
				{
					goto L2;
				}
				balance[i].insert(balance[i].begin(), tmp.time, true);
				cars[tmp.index].starttime = timeStart;//对处理过的car的starttime赋值
				cars[tmp.index].starttimeAnswer = cars[tmp.index].starttime;
				qCar.erase(qCar.begin());
			}
			else
				continue;//如果
		}
		L2:timeStart++;
	}
	L1:cout << endl;
}

void Scheduler::getPathByTime() 
{
	int num = 0;
	static int flagnum = 0;
	static int flag[100] = { 0 };
	static int flag_road[120] = { 0 };

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	int timeCar = 0;

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed, timeCar);

		cars[i].time = timeCar;

		//统计车辆情况，每100辆车更新一次jamDegree的矩阵
		num++;
		flagnum++;
		if (num == 200)
		{
			num = 0;
			graph.upDateJam();
		}
		for (int i = 0; i < pathCross.size(); i++)
		{
			flag[pathCross.at(i) - 1]++;//记录64个cross的使用情况
		}
		//将统计的情况放到jamDegreeTmp的矩阵中
		for (int i = 0, j = 1; j < pathCross.size(); i++, j++)
		{
			graph.jamDegreeTmp[pathCross.at(i) - 1][pathCross.at(j) - 1]++;
		}

		//cross矩阵转road矩阵
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;

	}
}

void Scheduler::getPathByTime_dynamic()
{
	int num = 0;

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

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
			//assert(pathRoad[j] != 0);
		}

		qCar[i].path = pathRoad;
		cars[qCar[i].index].path = qCar[i].path;	//将qCar得到的路径赋值到cars的path变量中
	}
}

bool CompDirMap(const Car &a, const Car &b)
{
	return a.dirMap > b.dirMap;
}

void Scheduler::getTimeByDir(int para)
{
	std::deque<Car> carsDeque;//此时间片待出发的车 //deque是分配在堆中的，所以此处临时deque不会造成栈溢出
	int timeStart = 1;//出发时间，从1开始安排

	for (int i = 0; i < num_Cars; ++i)
	{
		int x1, y1, x2, y2;
		x1 = (cars[i].idCrossFrom - 1) % 8;
		y1 = (cars[i].idCrossFrom - 1) / 8;
		x2 = (cars[i].idCrossTo - 1) % 8;
		y2 = (cars[i].idCrossTo - 1) / 8;
		if ((x2 - x1) <= 0 && (y2 - y1) <= 0)
		{
			cars[i].dirMap = 4;
		}
		else if ((x2 - x1) >= 0 && (y2 - y1) >= 0)
		{
			cars[i].dirMap = 3;
		}
		else if ((x2 - x1) >= 0 && (y2 - y1) <= 0)
		{
			cars[i].dirMap = 2;
		}
		else
		{
			cars[i].dirMap = 1;
		}

	}
	sort(speedType.begin(), speedType.end(), Comp);
	for (auto speed : speedType)
	{
		assert(carsDeque.size() == 0);//使用前确保deque为空
		for (int i = 0; i < num_Cars; ++i)
		{
			if (cars[i].speed == speed)
			{
				carsDeque.push_back(cars[i]);
			}
		}
		//std::sort(carsDeque.begin(), carsDeque.end(), less_time);//将deque中的车按照运行时间升序排列
		std::sort(carsDeque.begin(), carsDeque.end(), CompDirMap);//将deque中的车按照方向分为两类
		//当然如果不排序也是有一定道理的
		int loadCur;//记录当前负载
		while (carsDeque.size() > 0)
		{
			loadCur = 0;
			while (loadCur < para)
			{
				if (carsDeque.size() == 0)
				{
					break;
				}
				Car carTime = carsDeque.front();
				carsDeque.pop_front();
				loadCur += carTime.time;
				cars[carTime.index].starttimeAnswer = timeStart;
				cars[carTime.index].starttime = cars[carTime.index].starttimeAnswer;
			}
			timeStart++;
		}
	}
	//防止starttime早于计划时间
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].starttimeAnswer < cars[i].plantime)
		{
			cars[i].starttimeAnswer = cars[i].plantime;
		}
	}
}

//拥挤度map的两个操作，可以发车返回true，不能发车返回false
bool Scheduler::judgement(map<string, float >& mapForJamDegree, vector<int> path)
{
	float threshold = 0.90;
	map<string, float>::iterator iter1;
	for (int i = 0; i < path.size(); i++)
	{
		if (i > 2)
			break;//只判断前2条road，如果不拥堵则直接返回true，可以加车
		iter1 = mapForJamDegree.find(to_string(path[i]));
		if (iter1 == mapForJamDegree.end())//如果第i条路不存在map中或这存在但是其拥堵程度小于阈值，则说明path中第i条road可以加车
			continue;//继续判断path的下一条road
		else if (mapForJamDegree[to_string(path[i])] < threshold)
			continue;
		else
			return false;//发现这条路存在拥堵，则整个path都是由堵车风险的，直接判断不能加车
	}
	return true;//path全部检查完毕后返回可以加车
}

//如果map中存在此键，则更新；不存在，则插入新键值
void Scheduler::mapUpdate(map<string, float > &mapForJamDegree, int RoadId, float percent)
{
	map<string, float>::iterator iter2;
	iter2 = mapForJamDegree.find(to_string(RoadId));//搜索键是否在map中
	if (iter2 == mapForJamDegree.end())//如果不存在
	{
		pair<string, float> value(string(to_string(RoadId)), percent);//新键键值对
		mapForJamDegree.insert(value);//将键值对插入map中
	}
	else
	{
		iter2->second = percent;//更新实值，由于iter2已经进行了find，所以这里当找到了，直接对其value更新
	}

}

int Scheduler::getPartition(vector<Car> &reorderCar, int begin, int end)
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

void Scheduler::quicksort(vector<Car> &reorderCar, int begin, int end)
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

void Scheduler::reorderCars(vector<Car> &reorderCar)
{
	reorderCar.clear();

	for (int i = 0; i < num_Cars; i++)
	{
		reorderCar.push_back(cars[i]);//将id顺序的车辆放到qcar的vector中
	}

	int begin = 0;
	int end = reorderCar.size() - 1;
	quicksort(reorderCar, begin, end);
}
