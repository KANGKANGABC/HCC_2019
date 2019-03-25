#include "Scheduler.h"
#include "fstream"

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

int Scheduler::getParaByScheduler()
{
	std::map<int, int> mapResult;
	int para = 80;
	int timeMax = INT_MAX;
	ReOrderStartBySpeed(para);
	//getPathByTime();//获得车辆的路径信息
	reorderCars();
	//getPathByTime();//获得车辆的路径信息
	getPathByTime_dynamic();

	for (int i = 0; i < 15; ++i)//迭代20次
	{
		ReOrderStartBySpeed(para);
		int time = getSysTime();
		if (time == false)
			time = INT_MAX;
		mapResult.insert(pair<int, int>(time,para));
		para -= 4;
	}
	for (auto &v : mapResult)
	{
		PRINT("result:%d para:%d\n", v.first,v.second);
	}
	map<int, int>::iterator it;
	it = mapResult.begin();
	//it++;
	para = it->second;
	ReOrderStartBySpeed(para);
	/*
	int timeFinal = getSysTime();
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].timeArrived > (timeFinal - 20))
		{
			cars[i].starttime = cars[i].starttime - 20;
			cars[i].starttimeAnswer = cars[i].starttime;
		}
	}
	*/
	int time = getSysTime();
	PRINT("timeFinal:%d\n",time);

	return para;
}

int Scheduler::getPathByScheduler(int w)
{
	std::map<int, int> mapResult;
	int para = 80;
	int timeMax = INT_MAX;
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
	//it++;
	para = it->second;
	ReOrderStartBySpeed(para);
	getPath();
	int time = getSysTimeChangePath(w);
	//getSysTime();
	int timeFinal = getSysTime();
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

	return para;
}

int Scheduler::getSysTime()
{
	time_Scheduler = 0;
	num_CarsScheduling = num_Cars;
	carsWaitInGarage.clear();
	for (int i = 0; i < num_Roads; ++i)
	{
		int idLaneStart = 0;
		if (roads[i].isDuplex)
		{
			idLaneStart = roads[i].channel;
		}
		for (int j = 0; j < idLaneStart + roads[i].channel; ++j)
		{
			if (roads[i].lane[j].laneCar.size() > 0)
			{
				roads[i].lane[j].laneCar.clear();
			}
		}
	}
	while (num_CarsScheduling > 0)
	{
		//PRINT("***********time_Scheduler:%d************\n", time_Scheduler);//打印系统时间

		/*第一步：先处理所有道路上的车辆，进行遍历扫描*/
		driveAllCarsJustOnRoadToEndState();

		/*第二步：先处理所有道路上的车辆，进行遍历扫描*/
		while (1)//终止条件为：一个循环后，没有任何车被调度
		{
			bool isWorkingCross = false;//标志变量，如果一个循环后没有任何一辆车被调度，则退出循环
			for (int i = 0; i < num_Crosses; ++i)////按照升序调度所有路口
			{
				int idCross = crosses[i].id;//获得路口ID
				while (1)//循环调度路口四个方向的车，直到全部车辆完成调度，或者阻塞
				{
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
					if (!isWorkingRoad)
						break;
				}
			}
			if (!isWorkingCross)//如果一个循环后没有任何一辆车被调度，则退出调度循环
				break;
		}

		driverCarInGarage();
		if (!putAllCarStatus())//输出所有车的状态
			return false;//发生死锁
		//putAllRoadStatus();
		time_Scheduler++;//更新调度器时间
		//putAllRoadStatus();
	}
	return time_Scheduler;
}

int Scheduler::getSysTimeChangePath(int para)
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);
	time_Scheduler = 0;
	num_CarsScheduling = num_Cars;
	carsWaitInGarage.clear();
	for (int i = 0; i < num_Roads; ++i)
	{
		int idLaneStart = 0;
		if (roads[i].isDuplex)
		{
			idLaneStart = roads[i].channel;
		}
		for (int j = 0; j < idLaneStart + roads[i].channel; ++j)
		{
			if (roads[i].lane[j].laneCar.size() > 0)
			{
				roads[i].lane[j].laneCar.clear();
			}
		}
	}
	while (num_CarsScheduling > 0)
	{
		//PRINT("***********time_Scheduler:%d************\n", time_Scheduler);//打印系统时间

		/*第一步：先处理所有道路上的车辆，进行遍历扫描*/
		driveAllCarsJustOnRoadToEndState();

		/*第二步：先处理所有道路上的车辆，进行遍历扫描*/
		while (1)//终止条件为：一个循环后，没有任何车被调度
		{
			bool isWorkingCross = false;//标志变量，如果一个循环后没有任何一辆车被调度，则退出循环
			for (int i = 0; i < num_Crosses; ++i)////按照升序调度所有路口
			{
				int idCross = crosses[i].id;//获得路口ID
				while (1)//循环调度路口四个方向的车，直到全部车辆完成调度，或者阻塞
				{
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
					if (!isWorkingRoad)
						break;
				}
			}
			if (!isWorkingCross)//如果一个循环后没有任何一辆车被调度，则退出调度循环
				break;
		}

		driverCarInGarageDynamic(graph, para);
		if (!putAllCarStatus())//输出所有车的状态
			return false;//发生死锁
		putAllRoadStatus();
		time_Scheduler++;//更新调度器时间
		//putAllRoadStatus();
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
		if (index >= cars[car.id - 10000].plantime)
			cars[car.id - 10000].starttime = index;
		else
			cars[car.id - 10000].starttime = cars[car.id - 10000].plantime;
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
		if (index >= cars[car.id - 10000].plantime)
			cars[car.id - 10000].starttime = 2 * para + index;
		else
			cars[car.id - 10000].starttime = 2 * para + cars[car.id - 10000].plantime;
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
		if (index >= cars[car.id - 10000].plantime)
			cars[car.id - 10000].starttime = 4 * para + index;
		else
			cars[car.id - 10000].starttime = 4 * para + cars[car.id - 10000].plantime;
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
		if (index >= cars[car.id - 10000].plantime)
			cars[car.id - 10000].starttime = 6 * para + index;
		else
			cars[car.id - 10000].starttime = 6 * para + cars[car.id - 10000].plantime;
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
	if (idCrossTarget == roads[idRoadTarget - 5000].idTo)
	{
		idLaneStart = roads[idRoadTarget - 5000].channel;
	}
	for (int i = idLaneStart; i < idLaneStart + roads[idRoadTarget - 5000].channel; ++i)
	{
		if (roads[idRoadTarget - 5000].lane[i].laneCar.size() != 0)
		{
			if (roads[idRoadTarget - 5000].lane[i].laneCar[roads[idRoadTarget - 5000].lane[i].laneCar.size() - 1].location > 1)//留下一个空位
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
	if (roads[idRoadTarget - 5000].lane[idLaneTarget].laneCar.size() == 0)
	{
		locationTarget = std::min(car.speed, roads[idRoadTarget - 5000].speed);
	}
	else
	{
		Car carNext = roads[idRoadTarget - 5000].lane[idLaneTarget].laneCar[roads[idRoadTarget - 5000].lane[idLaneTarget].laneCar.size() - 1];//目标车道的最后一辆车
		if (carNext.location > std::min(car.speed, roads[idRoadTarget - 5000].speed))//不形成阻挡
		{
			locationTarget = std::min(car.speed, roads[idRoadTarget - 5000].speed);
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
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//将该车加入对应道路,对应车道,加入队尾

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
				}
			}
		}
	}
}

int Scheduler::driveCarNew(Car car)
{
	int indexCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size() - 1;
	if (roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size() == 1)//该车为该车道的第一辆车
	{
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//车正常行驶
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
	}
	else//该车前面有车
	{
		Car carNext = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1];
		if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) < carNext.location)
		{//前面的车不形成阻挡
			roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//车正常行驶
			roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
			roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
		}
		else
		{//前面的车形成阻挡
			if (carNext.status == FINESHED)
			{
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location = carNext.location - 1;//行驶到前车的后一个位置
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
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
	//判断此车会不会通过路口
	if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) <= roads[car.idCurRoad - 5000].length)//不会驶出路口
	{
		if (indexCar == 0)//是第一辆车
		{
			roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//车正常行驶
			roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
			roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
		}
		else//不是第一辆车
		{
			Car carNext = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1];
			if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) < carNext.location)
			{//前面的车不形成阻挡
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//车正常行驶
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{//前面的车形成阻挡
				if (carNext.status == FINESHED)
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location = carNext.location - 1;//行驶到前车的后一个位置
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
				}
				else
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//车标记为WAITTING
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
			if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
				idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
			else
				idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
			//根据假设AA，此时可能有车辆驶入终点
			if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
			{
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = WAITTING;
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = DD;//从NONE改为DD，直行到达终点
			}
			else
			{
				int idNextRoad = car.path[0];//此车即将驶入的道路
				int idCurRoad = car.idCurRoad;//此车当前道路

				int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
				if (disNextRoad == 0)//可行驶距离为0，则停在当前路口
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
				}
				else
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//此车变为等待状态
				}
			}
		}
		else//不是第一辆车
		{
			Car carNext = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1];
			if (carNext.status == FINESHED)
			{
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location = carNext.location - 1;//行驶到前车的后一个位置
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{
				//此车也将行驶出路口，需要判断此车在路口的方向
				//判断车的方向
				int idNextCross = 0;
				if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
					idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
				else
					idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
				//根据假设AA，此时可能有车辆驶入终点
				if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = DD;//从NONE改为DD，直行到达终点
				}
				else
				{
					int idNextRoad = car.path[0];//此车即将驶入的道路
					int idCurRoad = car.idCurRoad;//此车当前道路
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//此车变为等待状态
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
	if (idCrossTarget == roads[idRoadTarget - 5000].idTo)
	{
		idLaneStart = roads[idRoadTarget - 5000].channel;
	}
	for (int i = idLaneStart; i < idLaneStart + roads[idRoadTarget - 5000].channel; ++i)
	{
		if (roads[idRoadTarget - 5000].lane[i].laneCar.size() != 0)
		{
			if (roads[idRoadTarget - 5000].lane[i].laneCar[roads[idRoadTarget - 5000].lane[i].laneCar.size() - 1].location > 1)
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
		//cars[car.id - 10000].starttime += 1;
		return false;
	}

	int locationTarget = 0;
	if (roads[idRoadTarget - 5000].lane[idLaneTarget].laneCar.size() == 0)
	{
		locationTarget = std::min(car.speed, roads[idRoadTarget - 5000].speed);
	}
	else
	{
		Car carNext = roads[idRoadTarget - 5000].lane[idLaneTarget].laneCar[roads[idRoadTarget - 5000].lane[idLaneTarget].laneCar.size() - 1];//目标车道的最后一辆车
		if (carNext.location > std::min(car.speed, roads[idRoadTarget - 5000].speed))//不形成阻挡
		{
			locationTarget = std::min(car.speed, roads[idRoadTarget - 5000].speed);
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
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	int indexCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();//该车为末尾
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//将该车加入对应道路,对应车道,加入队尾

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
	int idStartLane = 0;//如果cross为道路的入方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idTo == crosses[idCross - 1].id)//如果cross为道路的入方向
	{
		idStartLane = roads[idRoad - 5000].channel;
		assert(roads[idRoad - 5000].isDuplex == 1);//如果cross为road的出方向，但是却不是双向道，那么很可能路径规划错误
	}
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() < roads[idRoad - 5000].length)
		{
			//这里有问题，车的数量可能小于车道长度，但是最后一辆车有可能堵在最后一位
			if (roads[idRoad - 5000].lane[j].laneCar.size() > 0)
			{
				Car carNext = roads[idRoad - 5000].lane[j].laneCar[roads[idRoad - 5000].lane[j].laneCar.size() - 1];
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
			Car carNext = roads[idRoad - 5000].lane[j].laneCar[roads[idRoad - 5000].lane[j].laneCar.size() - 1];
			if (carNext.status == WAITTING)
				return j;
		}
	}
	return idStartLane + roads[idRoad - 5000].channel - 1;//如果无空位返回最后一个车道
}

bool Scheduler::isBeDD(int idRoad, int idCross)//注意这里的ID不需要减1
{
	if (idRoad == -1)//如果冲突方向无道路，则任务无冲突车辆
		return false;
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idFrom == crosses[idCross - 1].id)//如果cross为道路的出方向
	{
		idStartLane = roads[idRoad - 5000].channel;
		if (roads[idRoad - 5000].isDuplex != 1)
			return false;
	}
	//原本以为需要判断所有车道，现在只判断优先级最高的车道
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == DD)
		{
			//只有等待状态的车会形成冲突
			if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
				return true;//存在左转车辆
		}

	}
	return false;
}

bool Scheduler::isBeLEFT(int idRoad, int idCross)//注意这里的ID不需要减1
{
	if (idRoad == -1)//如果冲突方向无道路，则任务无冲突车辆
		return false;
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idFrom == crosses[idCross - 1].id)//如果cross为道路的出方向
	{
		idStartLane = roads[idRoad - 5000].channel;
		if (roads[idRoad - 5000].isDuplex != 1)
			return false;
	}
	//原本以为需要判断所有车道，现在只判断优先级最高的车道
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == LEFT)
		{
			//只有等待状态的车会形成冲突
			if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
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
	int disCurRoad = roads[car.idCurRoad - 5000].length - car.location;//当前道路可以行驶的距离
	int disNextRoadMax = std::min(roads[idNextRoad - 5000].speed, car.speed);//下一道路可以行驶的最大距离
	assert(disCurRoad >= 0);
	assert(disNextRoadMax >= 0);
	if (disCurRoad >= disNextRoadMax)
		return 0;
	else
		return disNextRoadMax - disCurRoad;
}

void Scheduler::driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location)
{
	assert(location > 0);
	std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);	//将该车从当前lane删除,该车肯定是当前lane的第一辆车
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//该车路口状态设置为NONE,代表已经驶离路口
	car.status = FINESHED;//该车调度完成，等待下一时间片再行驶
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//将该车加入下一个lane,加入队尾
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	assert(crosses[idCross - 1].roadID[dir] != -1);//不可能进入无效路径
	int idNextCross = 0;
	if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
		idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
	else
		idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
	int idNextRoad = car.path[0];//获取目标道路
	int idNextLane = isCanEnter(idNextRoad, idNextCross);

	int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
	assert(disNextRoad!=0);
	if (disNextRoad == 0)//可行驶距离为0，则停在当前路口
	{
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
		return true;
	}
	else
	{
		int sizeLaneCar = roads[idNextRoad - 5000].lane[idNextLane].laneCar.size();
		if (sizeLaneCar > 0)
		{
			Car carNext = roads[idNextRoad - 5000].lane[idNextLane].laneCar[sizeLaneCar - 1];
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
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
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
		addCar(car, car.id - 10000);
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
		addCar(car, car.id - 10000);
	}
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].starttime == time_Scheduler && cars[i].status == SLEEPING)
		{
			int timeCar = 0;
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
		PRINT("Dead Lock Cross ID:");
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
	for (int i = 0; i < num_Roads; ++i)
	{
		float perRoad = 0;
		float threshold = 0.8;
		if (roads[i].isDuplex)
		{
			for (int j = 0; j < roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idFrom - 1][roads[i].idTo - 1] = perRoad;//更新拥堵情况矩阵
			if (perRoad > threshold)
				//PRINT("crossID:%d  %f\n", roads[i].idFrom, perRoad);
			perRoad = 0;
			for (int j = roads[i].channel; j < 2 * roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idTo - 1][roads[i].idFrom - 1] = perRoad;//更新拥堵情况矩阵
			if (perRoad > threshold)
				//PRINT("crossID:%d  %f\n", roads[i].idTo, perRoad);
			perRoad = 0;
		}
		else
		{
			for (int j = 0; j < roads[i].channel; ++j)
			{
				float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
				perRoad += per;
			}
			perRoad = perRoad / roads[i].channel;
			graphRoadStatusByDS[roads[i].idFrom - 1][roads[i].idTo - 1] = perRoad;//更新拥堵情况矩阵
			if (perRoad > threshold)
				//PRINT("crossID:%d  %f\n", roads[i].idFrom, perRoad);
			perRoad = 0;
		}
	}
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
	if (roads[idRoad - 5000].lane[idChannel].laneCar.size() != 0)
	{
		for (int i = 0; i < roads[idRoad - 5000].lane[idChannel].laneCar.size(); ++i)
		{
			if (roads[idRoad - 5000].lane[idChannel].laneCar[i].status == WAITTING)//只处理等待状态的车
			{
				Car car = roads[idRoad - 5000].lane[idChannel].laneCar[i];
				if (car.location + std::min(roads[idRoad - 5000].speed, car.speed) <= roads[idRoad - 5000].length)//不会驶出路口
				{
					//只处理行驶后不通过路口的车
					if (i != 0)//如果该车不是第一辆车
					{
						Car carNext = roads[idRoad - 5000].lane[idChannel].laneCar[i - 1];
						if (car.location + std::min(roads[idRoad - 5000].speed, car.speed) < carNext.location)
						{
							//前车不形成阻挡
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location += std::min(roads[idRoad - 5000].speed, car.speed);//车正常行驶
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
						else if (carNext.status == FINESHED)
						{
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location = carNext.location - 1;//车正常行驶
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
					}
					else
					{
						//前车不形成阻挡
						roads[idRoad - 5000].lane[idChannel].laneCar[i].location += std::min(roads[idRoad - 5000].speed, car.speed);//车正常行驶
						roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
						roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
					}
				}
				else 
				{
					//只处理行驶后不通过路口的车
					if (i != 0)//如果该车不是第一辆车
					{
						Car carNext = roads[idRoad - 5000].lane[idChannel].laneCar[i - 1];
						if (carNext.status == FINESHED)
						{
							roads[idRoad - 5000].lane[idChannel].laneCar[i].location = carNext.location - 1;//车正常行驶
							roads[idRoad - 5000].lane[idChannel].laneCar[i].status = FINESHED;//车标记为终止状态
							roads[idRoad - 5000].lane[idChannel].laneCar[i].dirCross = NONE;
						}
					}
					else
					{
						if (car.path.size() > 0)
						{
							int idNextRoad = car.path[0];//此车即将驶入的道路
							int idCurRoad = car.idCurRoad;//此车当前道路

							int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
							if (disNextRoad == 0)//可行驶距离为0，则停在当前路口
							{
								roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
								roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//车标记为终止状态
								roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
							}
						}
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
			//assert(pathRoad[j] != 0);
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
					cars[car.id - 10000].starttime = timeStart;
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
				cars[carLongTime.id - 10000].starttime = timeStart;
				cars[carShortTime.id - 10000].starttime = timeStart;
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
				cars[tmp.id - 10000].starttime = timeStart;//对处理过的car的starttime赋值
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

void Scheduler::getPathByTime_reorderCars()
{
	int num = 0;
	static int flagnum = 0;
	static int flag[100] = { 0 };
	static int flag_road[120] = { 0 };

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(qCar[i].idCrossFrom, qCar[i].idCrossTo, qCar[i].speed);

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
		
		//统计road的情况
		for (int i = 0; i < pathRoad.size(); i++)
		{
			flag_road[pathRoad.at(i) - 5000]++;//记录road的使用情况
		}


		//统计cross的情况
		if (flagnum == 200)
		{
			ofstream oFile;
			oFile.open("testcross100.csv", ios::out | ios::trunc);
			for (int i=0; i < 100; i++)
			{
				oFile << flag[i] << endl;
			}

			oFile.close();

			for (int i = 0; i < 100; i++)
			{
				flag[i] = 0;
			}

			ofstream oFile1;
			oFile1.open("testroad100.csv", ios::out | ios::trunc);
			for (int i = 0; i < 120; i++)
			{
				oFile1 << flag_road[i] << endl;
			}
			oFile1.close();

			for (int i = 0; i < 120; i++)
			{
				flag_road[i] = 0;
			}
		}
		
		//写出100~200辆车的统计情况
		if (flagnum == 400)
		{
			ofstream oFile2;
			oFile2.open("testcross200.csv", ios::out | ios::trunc);
			for (int i = 0; i < 100; i++)
			{
				oFile2 << flag[i] << endl;
			}

			oFile2.close();


			ofstream oFile3;
			oFile3.open("testroad200.csv", ios::out | ios::trunc);
			for (int i = 0; i < 120; i++)
			{
				oFile3 << flag_road[i] << endl;
			}

			oFile3.close();
		}
		qCar[i].path = pathRoad;
		cars[qCar[i].id - 10000].path = qCar[i].path;	//将qcars得到的路径赋值到cars的path变量中
	}
}


void Scheduler::getPathByTime_dynamic()
{
	int num = 0;
	//static int flag[100] = { 0 };

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
		cars[qCar[i].id - 10000].path = qCar[i].path;	//将qcars得到的路径赋值到cars的path变量中
	}
}

void Scheduler::swap(int i, int j)
{
	Car tmp;
	tmp = qCar[i];
	qCar[i] = qCar[j];
	qCar[j] = tmp;
}

void Scheduler::quicksort(int begin, int end)
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
void Scheduler::reorderCars()
{
	for (int i = 0; i < num_Cars; i++)
	{
		qCar.push_back(cars[i]);//将id顺序的车辆放到qcar的vector中
	}

	int begin = 0;
	int end = qCar.size() - 1;
	quicksort(begin, end);
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
				cars[carTime.id - 10000].starttimeAnswer = timeStart;
				cars[carTime.id - 10000].starttime = cars[carTime.id - 10000].starttimeAnswer;
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
