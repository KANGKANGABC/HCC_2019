#include "Scheduler.h"

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

}

Scheduler::~Scheduler()
{
}

int Scheduler::getSysTime()
{
	while (num_CarsScheduling > 0)
	{
		PRINT("***********time_Scheduler:%d************\n", time_Scheduler);//打印系统时间

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
					for (int j = 0; j < 4; ++j)//这里按要求是根据道路id进行升序调度
					{
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
											switch (roads[idRoad - 5000].lane[m].laneCar[0].dirCross)
											{
											case NONE:
												break;
											case DD://直行>左转>右转
												dirTarget = getDirByRoadCrossDir(idCross,idRoad) + 2;//目标方向
												if (dirTarget > 3) dirTarget -= 4;//修正方向
												if (isCanDriveToNextRoad(car, dirTarget, idCross))
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
													driveAllCarsJustOnOneChannelToEndState(idRoad, idCross,m);
												}
												//判断转入的road是否可以行驶

												break;
											case LEFT://左转>右转
												//判断即将转入的方向是否有直行进入的车辆
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) - 1;//冲突方向
												if (dirConflict < 0) dirConflict += 4;//修正方向
												if (!isBeDD(crosses[i].roadID[dirConflict], i))
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
												}
												break;
											case RIGHT://右转优先级最低
												//判断即将转入的方向是否有直行进入的车辆
												dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 1;//冲突方向
												if (dirConflict > 3) dirConflict -= 4;//修正方向
												if (!isBeDD(crosses[i].roadID[dirConflict], i))
												{
													dirConflict = getDirByRoadCrossDir(idCross, idRoad) + 2;//冲突方向
													if (dirConflict > 3) dirConflict -= 4;//修正方向
													//判断即将转入的方向是否有左转进入的车辆
													if (!isBeLEFT(crosses[i].roadID[dirConflict], i))
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
													}
												}
												break;
											default:
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
		driverCarInGarage();//车库中的上路行驶
		putAllCarStatus();//输出所有车的状态
		putAllRoadStatus();
		time_Scheduler++;//更新调度器时间
	}
	return time_Scheduler;
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
					driveCar(roads[i].lane[j].laneCar[m], m);
					//roads[i].lane[j] = lane;//这里一定要将lane写回，否则车辆调度无效
					//这里不要写回，写回容易误操作，只需要保证更新car的状态是通过road->lane->car的方式索引即可
					//此处有问题，如果一个车道有两辆车，但是第一辆车被开到下一车道，如果车道信息没有及时更新
					//会出现误操作，索引到并不存在的第二辆车
					
				}
			}
		}
	}

}

bool Scheduler::driveCar(Car car, int indexCar)
{
	//这里假设AA：如果某辆车从路口驶入下一道路，不可能在一个时间片内驶完下一道路全程
	//也就是说只有处于NONE状态WAITTING的车才有可能即将到达终点
	if (car.dirCross == NONE)//该车不是在路口等待
	{
		assert(indexCar != -1);//此情况下indexCar不能为-1
		assert(roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size()!=0);//断言该车道上至少有自己一辆车
		assert(indexCar < roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size());//断言车的序号小于车道上车数量
		if (indexCar == 0 || roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size()== 1)//该车为该车道的第一辆车，且上个时间片不准备通过路口
		{
			//判断此车会不会通过路口
			if (car.location + std::min(roads[car.idCurRoad - 5000].speed, car.speed) <= roads[car.idCurRoad - 5000].length)//不会驶出路口
			{
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location += std::min(roads[car.idCurRoad - 5000].speed, car.speed);//车正常行驶
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
			}
			else
			{
				//此车也将行驶出路口，需要判断此车在路口的方向
				//判断车的方向
				int idNextCross = 0;
				if(car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
					idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
				else
					idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
				//根据假设AA，此时可能有车辆驶入终点
				if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
				{
					num_CarsScheduling -= 1;//正在调度的车辆数减一
					std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);//删除该道路第一辆车
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
				//判断此车会不会通过路口
				if (carNext.status == FINESHED)
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].location = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].location - 1;//行驶到前车的后一个位置
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = FINESHED;//车标记为终止状态
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = NONE;
				}
				else
				{
					roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//车标记为WAITTING
				}
				/*
				else
				{
					//此车也将行驶出路口,那么判断此车在路口的方向
					int idNextCross = 0;
					if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
						idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
					else
						idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
					//根据假设AA，此时可能有车辆驶入终点
					if (idNextCross == car.idCrossTo)//如果此车将要驶出出口
					{
						num_CarsScheduling -= 1;//正在调度的车辆数减一
						std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);//删除该道路第一辆车
					}
					else
					{
						int idNextRoad = car.path[0];//此车即将驶入的道路
						int idCurRoad = car.idCurRoad;//此车当前道路
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
						roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar].status = WAITTING;//此车变为等待状态
					}
				}
				*/
			}
		}
	}
	else//该车准备驶出路口
	{
		assert(car.status == WAITTING);//只有处于等待状态的车才能出路口
		int idNextCross = 0;
		if (car.idCurLane >= roads[car.idCurRoad - 5000].channel)//逆向
			idNextCross = roads[car.idCurRoad - 5000].idFrom;//此车即将驶入的路口
		else
			idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
		assert(idNextCross != car.idCrossTo);
		int idNextRoad = car.path[0];//获取目标道路
		int idNextLane = isCanEnter(idNextRoad, idNextCross);
		if (idNextLane != -1)//如果该道路可加入车
		{
			int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
			if (disNextRoad == 0)//可行驶距离为0，则停在当前路口
			{
				//此时比较特殊，因为没有发生Road变化，所以car依然在当前lane，但是其location需要更新
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].location = roads[car.idCurRoad - 5000].length;
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].status = FINESHED;//该车调度完成，等待下一时间片再行驶
				roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[0].dirCross = NONE;
				return true;
			}
			else
			{
				if (roads[idNextRoad-5000].lane[idNextLane].laneCar.size() != 0)//如果该车道有车
				{
					//判断对应车道车的位置
					Car carNext = roads[idNextRoad - 5000].lane[idNextLane].laneCar[roads[idNextRoad - 5000].lane[idNextLane].laneCar.size() - 1];
					if (disNextRoad < carNext.location)//不形成阻挡
					{
						driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//行驶到下个路口
						return true;
					}
					else//形成阻挡
					{
						if (carNext.status = FINESHED)
						{
							driverToNextRoad(car, idNextRoad, idNextLane, carNext.location - 1);//行驶到下个路口，前车之后
							assert(carNext.location >= 1);
							return true;
						}
						//如果前车处于等待状态，那么此车也不行驶，继续等待
					}
				}
				else//如果该车道没有阻挡
				{
					driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//行驶到下个路口
					return true;
				}
			}
		}
		//如果目标车道无法驶入，保持WAITTING状态
	}
	return false;
}

void Scheduler::addCar(Car car, int i)
{
	assert(car.status == SLEEPING);//只有SLEEPING状态的车可以加入地图行驶
	int idRoadTarget = car.path[0];//获取目标道路
	int idCrossTarget = car.idCrossFrom;//获得该车出发路口
	int idLaneTarget = isCanEnter(idRoadTarget, idCrossTarget);
	if (idLaneTarget != -1)//如果该道路可加入车
	{
		car.status = FINESHED;//切换car的状态
		car.idCurRoad = idRoadTarget;
		car.idCurLane = idLaneTarget;
		car.location = 0;
		car.dirCross = NONE;
		std::vector<int>::iterator itPath = car.path.begin();
		car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
		int indexCar = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();//该车为末尾
		roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//将该车加入对应道路,对应车道,加入队尾
		driveCar(car, indexCar);//car行驶 indexCar为-1，表示该车在lane中还没有位置
		cars[car.id - 10000].starttime = time_Scheduler;
		num_CarsPut += 1;
		//roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar[indexCar - 1].starttime = time_Scheduler;
		//记录实际出发时间
	}
	else//如果加入失败，则将出发时间延后一个1个时间片
	{
		cars[i].plantime += 1;
	}
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
		if (roads[idRoad - 5000].isDuplex != 1)
			return -1;
	}
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() < roads[idRoad - 5000].length)
		{
			//这里有问题，车的数量可能小于车道长度，但是最后一辆车有可能堵在最后一位
			if (roads[idRoad - 5000].lane[j].laneCar.size() > 0)
			{
				Car carNext = roads[idRoad - 5000].lane[j].laneCar[roads[idRoad - 5000].lane[j].laneCar.size() - 1];
				if (carNext.location == 0)
					continue;//修复此处重大bug，原本是return -1;
				else
					return j;
			}
			else
			{
				return j;//存在空位，可加入,返回车道id
			}
		}
		else
		{
			continue; 
		}
	}
	return -1;//不存在空位
}

bool Scheduler::isBeDD(int idRoad, int idCross)
{
	if (idRoad == -1)//如果冲突方向无道路，则任务无冲突车辆
		return true;
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idTo == crosses[idCross-1].id)//如果cross为道路的出方向
		idStartLane = roads[idRoad - 5000].channel;
	/*
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == DD)
				return true;//存在直行车辆
		}
	}
	*/
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

bool Scheduler::isBeLEFT(int idRoad, int idCross)
{
	if (idRoad == -1)//如果冲突方向无道路，则任务无冲突车辆
		return true;
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idTo == crosses[idCross-1].id)//如果cross为道路的出方向
		idStartLane = roads[idRoad - 5000].channel;
	/*
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == LEFT)
				return true;//存在左转车辆
		}
	}
	*/
	//原本以为需要判断所有车道，现在只判断优先级最高的车道
	if (roads[idRoad - 5000].lane[idStartLane].laneCar.size() != 0)
	{
		if (roads[idRoad - 5000].lane[idStartLane].laneCar[0].dirCross == LEFT)
		{
			//只有等待状态的车会形成冲突
			if(roads[idRoad - 5000].lane[idStartLane].laneCar[0].status == WAITTING)
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
	std::vector<Car>::iterator it = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.begin();
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.erase(it);	//将该车从当前lane删除,该车肯定是当前lane的第一辆车
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//该车路口状态设置为NONE,代表已经驶离路口
	car.status = FINESHED;//该车调度完成，等待下一时间片再行驶
	std::vector<int>::iterator itPath = car.path.begin();
	car.path.erase(itPath);//已经驶向下一个路口，所以删除path中第一项
	int size = roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.size();
	Lane lane = roads[car.idCurRoad - 5000].lane[car.idCurLane];
	roads[car.idCurRoad - 5000].lane[car.idCurLane].laneCar.push_back(car);//将该车加入下一个lane,加入队尾
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	if (crosses[idCross-1].roadID[dir] != -1)
	{
		int idNextLane = isCanEnter(crosses[idCross - 1].roadID[dir], idCross);
		if (idNextLane != -1)
		{
			bool result = driveCar(car, -1);//该车可以转向，但是不代表该车转向后不会因为有车阻挡然后WAITTING
			//实际上driverCar后可能有如下三种情况
			//1.成功行驶到下个路口
			//2.因为下个路口有车waitting,此车无法行驶
			//3.因为行驶里程不够，只能继续等在路口
			//if (car_original.idCurRoad != car.idCurRoad || car_original.location != car.location)//road发生变化或者location发生变化，视为转向成功
			if(result)
				return true;
			//第三种情况视为成功或者不成功，对结果应该影响不大
		}
	}
	return false;
}

void Scheduler::driveAllCarsJustOnOneRoadToEndState(int idRoad, int idCross)
{
	Road road = roads[idRoad - 5000];
	if (road.idTo = idCross)//如果该路口是road的出方向，那么遍历road的正方向
	{
		for (int j = 0; j < road.channel; ++j)
		{
			Lane lane = road.lane[j];
			for (int m = 0; m < lane.laneCar.size(); ++m)
			{
				driveCar(lane.laneCar[m], m);
			}
		}
	}
	else //遍历road的反方向
	{
		if (road.isDuplex)
		{
			for (int j = road.channel; j < 2 * road.channel; ++j)
			{
				Lane lane = road.lane[j];
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					driveCar(lane.laneCar[m], m);
				}
			}
		}
	}
}

void Scheduler::driverCarInGarage()
{
	for (int i = 0; i < num_Cars; ++i)
	{
		if (cars[i].plantime == time_Scheduler&&cars[i].status==SLEEPING)
			addCar(cars[i],i);
	}
}

void Scheduler::putCarStatus(Car car)
{
	if(car.path.size()!=0)
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  NextRoad:%d  Status:%d\n",
		car.id,car.idCrossFrom,car.idCrossTo,car.idCurRoad,car.idCurLane,car.location,car.path[0], car.status);//打印系统时间
	else
		PRINT("CarID:%d  idFrom:%d  idTo:%d idCurRoad:%d  idCurLane:%d  location:%d  Status:%d\n",
			car.id, car.idCrossFrom, car.idCrossTo, car.idCurRoad, car.idCurLane, car.location, car.status);//打印系统时间
}

void Scheduler::putAllCarStatus()
{
	for (int i = 0; i < num_Roads; ++i)//按道路ID升序进行调度
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			Lane lane = roads[i].lane[j];
			
			if (lane.laneCar.size() != 0)//先判断该车道是否有车
			{
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					putCarStatus(lane.laneCar[m]);
				}
			}
		}
	}
}

void Scheduler::putAllRoadStatus()
{
	int num_road_jam = 0;//堵住的道路数量，per高于0.7则认为堵车
	for (int i = 0; i < num_Roads; ++i)
	{
		for (int j = 0; j < roads[i].channel * (1 + roads[i].isDuplex); ++j)
		{
			float per = (float)roads[i].lane[j].laneCar.size() / (float)roads[i].length;
			if(per > 0.5)
				PRINT("ROAD ID:%d  CHANNEL ID:%d  PERCENT:%f\n",roads[i].id,j,per);
			if (per > 0.7)
				num_road_jam++;
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
			}
		}
	}
}

void Scheduler::getPath()
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
	}
}

void Scheduler::getPathByTime()
{
	static int num = 0;
	static int flag[100] = { 0 };

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed);

		//统计车辆情况，每100辆车更新一次jamDegree的矩阵
		num++;
		if (num == 200)
		{
			num = 0;
			graph.upDateJam();
			for (int i = 0; i < vexnum; i++)
			{
				flag[i] = 0;
			}
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

		//if (num == 100)
		//{
		//	ofstream oFile;
		//	oFile.open("test.csv", ios::out | ios::trunc);
		//	for (int i=0; i < 100; i++)
		//	{
		//		oFile << flag[i] << endl;
		//	}
		//	
		//	for (int i = 0; i < 64; i++)
		//	{
		//		flag[i] = 0;
		//	}
		//	graph.upDateJam();
		//	oFile.close();
		//}

		///*写出100~200辆车的统计情况*/
		//if (num == 200)
		//{
		//	ofstream oFile;
		//	oFile.open("test2.csv", ios::out | ios::trunc);
		//	for (int i = 0; i < 100; i++)
		//	{
		//		oFile << flag[i] << endl;
		//	}

		//	oFile.close();
		//}

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
	}

}

void Scheduler ::getPathByTime_dynamic()
{
	static int num = 0;
	//static int flag[100] = { 0 };

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);

	for (int i = 0; i < num_Cars; ++i)
	{
		vector<int> pathCross = graph.DijkstraNor(cars[i].idCrossFrom, cars[i].idCrossTo, cars[i].speed);

		num++;
		//定时更新交通拥堵邻接矩阵jamDegreeLongBefore
		if (num == 200)
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

		//if (num == 100)
		//{
		//	ofstream oFile;
		//	oFile.open("test.csv", ios::out | ios::trunc);
		//	for (int i=0; i < 100; i++)
		//	{
		//		oFile << flag[i] << endl;
		//	}
		//	
		//	for (int i = 0; i < 64; i++)
		//	{
		//		flag[i] = 0;
		//	}
		//	graph.upDateJam();
		//	oFile.close();
		//}

		///*写出100~200辆车的统计情况*/
		//if (num == 200)
		//{
		//	ofstream oFile;
		//	oFile.open("test2.csv", ios::out | ios::trunc);
		//	for (int i = 0; i < 100; i++)
		//	{
		//		oFile << flag[i] << endl;
		//	}

		//	oFile.close();
		//}

		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		cars[i].path = pathRoad;
	}

}