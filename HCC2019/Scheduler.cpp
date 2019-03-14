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
	Lane lane = roads[0].lane[0];
	time_Scheduler = 0;//调度器初始时间设置为0
}


Scheduler::~Scheduler()
{
}

int Scheduler::getSysTime()
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
				while (1)//循环调度路口四个方向的车，直到全部车辆完成调度，或者阻塞
				{
					bool isWorkingRoad = false;
					for (int j = 0; j < 4; ++j)
					{
						if (crosses[i].roadID[j] != -1)
						{
							int idRoad = crosses[i].roadID[j];//被调度的道路id
							int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
							if (roads[idRoad - 5000].idTo == crosses[i].id)//如果cross为道路的出方向
								idStartLane = roads[idRoad - 5000].channel;
							while (1)
							{
								bool isWorkingLane = false;
								for (int m = idStartLane; m < idStartLane + roads[idRoad - 5000].channel; ++m)//遍历所有lane
								{
									if (roads[idRoad - 5000].lane[m].laneCar.size() != 0)
									{
										Car car = roads[idRoad - 5000].lane[m].laneCar[0];
										int dirConflict = 0;
										int dirTarget = 0;
										switch (roads[idRoad - 5000].lane[m].laneCar[0].dirCross)
										{
										case NONE:
											break;
										case DD://直行>左转>右转
											dirTarget = j + 2;//目标方向
											if (dirTarget < 0) dirTarget += 4;//修正方向
											if (isCanDriveToNextRoad(car, dirTarget, idCross))
											{
												isWorkingCross = true;
												isWorkingRoad = true;
												isWorkingLane = true;
											}
										   //判断转入的road是否可以行驶

											break;
										case LEFT://左转>右转
											//判断即将转入的方向是否有直行进入的车辆
											dirConflict = j - 1;//冲突方向
											if (dirConflict < 0) dirConflict += 4;//修正方向
											if (!isBeDD(crosses[i].roadID[dirConflict], i))
											{
												dirTarget = j + 1;//目标方向
												if (dirTarget > 3) dirTarget -= 4;//修正方向
												if (isCanDriveToNextRoad(car, dirTarget, idCross))//判断转入的road是否可以行驶
												{
													isWorkingCross = true;
													isWorkingRoad = true;
													isWorkingLane = true;
												}
											}
											break;
										case RIGHT://右转优先级最低
											//判断即将转入的方向是否有直行进入的车辆
											dirConflict = j + 1;//冲突方向
											if (dirConflict > 3) dirConflict -= 4;//修正方向
											if (!isBeDD(crosses[i].roadID[dirConflict], i))
											{
												dirConflict = j + 2;//冲突方向
												if (dirConflict > 3) dirConflict -= 4;//修正方向
												//判断即将转入的方向是否有左转进入的车辆
												if (!isBeLEFT(crosses[i].roadID[dirConflict], i))
												{
													dirTarget = j - 1;//目标方向
													if (dirTarget < 0) dirTarget += 4;//修正方向
													if (isCanDriveToNextRoad(car, dirTarget, idCross))//判断转入的road是否可以行驶
													{
														isWorkingCross = true;
														isWorkingRoad = true;
														isWorkingLane = true;
													}
												}
											}
											break;
										default:
											break;
										}
									}
								}
								if (!isWorkingLane)
									break;
								else
								{
									//如果该此调度有车出路口，那么让该Road的所有lane行驶
									driveAllCarsJustOnOneRoadToEndState(idRoad, idCross);
								}
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
			Lane lane = roads[i].lane[j];
			if (lane.laneCar.size() != 0)//先判断该车道是否有车
			{
				for (int m = 0; m < lane.laneCar.size(); ++m)
				{
					driveCar(lane.laneCar[m], m);
				}
			}
		}
	}

}

void Scheduler::driveCar(Car car, int indexCar)
{
	//这里假设AA：如果某辆车从路口驶入下一道路，不可能在一个时间片内驶完下一道路全程
	//也就是说只有处于NONE状态WAITTING的车才有可能即将到达终点
	if (car.dirCross == NONE)//该车不是在路口等待
	{
		if (indexCar == 0)//该车为该车道的第一辆车，且上个时间片不准备通过路口
		{
			//判断此车会不会通过路口
			if (car.location + std::min(roads[car.idCurRoad].speed, car.speed) <= roads[car.idCurRoad].length)//不会驶出路口
			{
				car.location += std::min(roads[car.idCurRoad].speed, car.speed);//车正常行驶
				car.status = FINESHED;//车标记为终止状态
			}
			else
			{
				//此车也将行驶出路口，需要判断此车在路口的方向
				int idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
				//根据假设AA，此时可能有车辆驶入终点
				if (idNextCross = car.idCrossTo)//如果此车将要驶出出口
				{
					num_CarsScheduling -= 1;//正在调度的车辆数减一
					std::vector<Car>::iterator it = roads[car.idCurRoad].lane[car.idCurLane].laneCar.begin();
					roads[car.idCurRoad].lane[car.idCurLane].laneCar.erase(it);//删除该道路第一辆车
				}
				else
				{
					int idNextRoad = car.path[car.path.size() - 1];//此车即将驶入的道路
					int idCurRoad = car.idCurRoad;//此车当前道路
					car.dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
					car.status = WAITTING;//此车变为等待状态
				}
			}
		}
		else//该车前面有车
		{
			Car carNext = roads[car.idCurRoad].lane[car.idCurLane].laneCar[indexCar - 1];
			if (car.location + std::min(roads[car.idCurRoad].speed, car.speed) < carNext.location)
			{//前面的车不形成阻挡
				car.location += std::min(roads[car.idCurRoad].speed, car.speed);//车正常行驶
				car.status = FINESHED;//车标记为终止状态
			}
			else
			{//前面的车形成阻挡
				//判断此车会不会通过路口
				if (car.location + std::min(roads[car.idCurRoad].speed, car.speed) <= roads[car.idCurRoad].length)//不会驶出路口
				{
					if (carNext.status == FINESHED)
					{
						car.location = roads[car.idCurRoad].lane[car.idCurLane].laneCar[indexCar - 1].location - 1;//行驶到前车的后一个位置
						car.status = FINESHED;//车标记为终止状态
					}
					else
					{
						car.status = WAITTING;//车标记为WAITTING
					}
				}
				else
				{
					//此车也将行驶出路口,那么判断此车在路口的方向
					int idNextCross = roads[car.idCurRoad - 5000].idTo;//此车即将驶入的路口
					//根据假设AA，此时可能有车辆驶入终点
					if (idNextCross = car.idCrossTo)//如果此车将要驶出出口
					{
						num_CarsScheduling -= 1;//正在调度的车辆数减一
						std::vector<Car>::iterator it = roads[car.idCurRoad].lane[car.idCurLane].laneCar.begin();
						roads[car.idCurRoad].lane[car.idCurLane].laneCar.erase(it);//删除该道路第一辆车
					}
					else
					{
						int idNextRoad = car.path[car.path.size() - 1];//此车即将驶入的道路
						int idCurRoad = car.idCurRoad;//此车当前道路
						car.dirCross = getCrossDir(idCurRoad, idNextRoad, idNextCross);//设置路口方向
						car.status = WAITTING;//此车变为等待状态
					}
				}
			}
		}
	}
	else//该车准备驶出路口
	{
		int idNextRoad = car.path[car.path.size() - 1];//获取目标道路
		int idNextCross = roads[car.idCurRoad - 5000].idTo;//获得该车出发路口
		if (int idNextLane = isCanEnter(idNextRoad, idNextCross) != -1)//如果该道路可加入车
		{
			int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
			if (disNextRoad == 0)//可行驶距离为0，则停在当前路口
			{
				car.location = roads[car.idCurRoad].length;
				car.status = FINESHED;//该车调度完成，等待下一时间片再行驶
			}
			else
			{
				if (roads[idNextRoad].lane[idNextLane].laneCar.size() != 0)//如果该车道有车
				{
					//判断对应车道车的位置
					Car carNext = roads[idNextRoad].lane[idNextLane].laneCar[roads[idNextRoad].lane[idNextLane].laneCar.size() - 1];
					if (disNextRoad < carNext.location)//不形成阻挡
					{
						driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//行驶到下个路口
					}
					else//形成阻挡
					{
						if (carNext.status = FINESHED)
							driverToNextRoad(car, idNextRoad, idNextLane, carNext.location - 1);//行驶到下个路口，前车之后
						//如果前车处于等待状态，那么此车也不行驶，继续等待
					}
				}
				else//如果该车道没有阻挡
				{
					int disNextRoad = getCrossDistance(car, car.idCurRoad, idNextRoad);
					driverToNextRoad(car, idNextRoad, idNextLane, disNextRoad);//行驶到下个路口
				}
			}
		}
		//如果目标车道无法驶入，保持WAITTING状态
	}
}

void Scheduler::addCar(Car car)
{
	assert(car.status == SLEEPING);//只有SLEEPING状态的车可以加入地图行驶
	int idRoadTarget = car.path[car.path.size() - 1];//获取目标道路
	int idCrossTarget = car.idCrossFrom;//获得该车出发路口
	if (isCanEnter(idRoadTarget, idCrossTarget) != -1)//如果该道路可加入车
	{
		car.status = WAITTING;//切换car的状态
		driveCar(car, -1);//car行驶 indexCar为-1，表示该车在lane中还没有位置
	}
}

int Scheduler::isCanEnter(int idRoad, int idCross)
{
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idFrom == crosses[idCross].id)//如果cross为道路的入方向
		idStartLane = roads[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() < roads[idRoad - 5000].length)
		{
			return j;//存在空位，可加入,返回车道id
		}
	}
	return -1;//不存在空位
}

bool Scheduler::isBeDD(int idRoad, int idCross)
{
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idTo == crosses[idCross].id)//如果cross为道路的出方向
		idStartLane = roads[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == DD)
				return true;//存在直行车辆
		}
	}
	return false;
}

bool Scheduler::isBeLEFT(int idRoad, int idCross)
{
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (roads[idRoad - 5000].idTo == crosses[idCross].id)//如果cross为道路的出方向
		idStartLane = roads[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + roads[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (roads[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (roads[idRoad - 5000].lane[j].laneCar[0].dirCross == LEFT)
				return true;//存在左转车辆
		}
	}
	return false;
}

int Scheduler::getCrossDir(int idCurRoad, int idNextRoad, int idNextCross)
{
	int dirCurRoad = 0;//当前道路在路口的方向
	int dirNextRoad = 0;//即将驶入道路在路口的方向
	if (crosses[idNextCross].roadID_T == idCurRoad)
		dirCurRoad = 0;
	else if (crosses[idNextCross].roadID_R == idCurRoad)
		dirCurRoad = 1;
	else if (crosses[idNextCross].roadID_D == idCurRoad)
		dirCurRoad = 2;
	else if (crosses[idNextCross].roadID_L == idCurRoad)
		dirCurRoad = 3;

	if (crosses[idNextCross].roadID_T == idNextRoad)
		dirNextRoad = 0;
	else if (crosses[idNextCross].roadID_R == idNextRoad)
		dirNextRoad = 1;
	else if (crosses[idNextCross].roadID_D == idNextRoad)
		dirNextRoad = 2;
	else if (crosses[idNextCross].roadID_L == idNextRoad)
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
	int disNextRoadMax = std::min(roads[idNextRoad - 5000].length, car.speed);//下一道路可以行驶的最大距离
	if (disCurRoad >= disNextRoadMax)
		return 0;
	else
		return disNextRoadMax - disCurRoad;
}

void Scheduler::driverToNextRoad(Car car, int idNextRoad, int idNextLane, int location)
{
	std::vector<Car>::iterator it = roads[car.idCurRoad].lane[car.idCurLane].laneCar.begin();
	roads[car.idCurRoad].lane[car.idCurLane].laneCar.erase(it);	//将该车从当前lane删除,该车肯定是当前lane的第一辆车
	car.idCurRoad = idNextRoad;
	car.idCurLane = idNextLane;
	car.location = location;
	car.dirCross = NONE;//该车路口状态设置为NONE,代表已经驶离路口
	car.status = FINESHED;//该车调度完成，等待下一时间片再行驶
	roads[car.idCurRoad].lane[car.idCurLane].laneCar.push_back(car);//将该车加入下一个lane,加入队尾
}

bool Scheduler::isCanDriveToNextRoad(Car car, int dir, int idCross)
{
	if (crosses[idCross].roadID[dir] != -1)
	{
		if (int idNextLane = isCanEnter(crosses[idCross].roadID[dir] - 5000, idCross))
		{
			Car car_original= car;//记录car转向之前的状态
			driveCar(car, -1);//该车可以转向，但是不代表该车转向后不会因为有车阻挡然后WAITTING
			//实际上driverCar后可能有如下三种情况
			//1.成功行驶到下个路口
			//2.因为下个路口有车waitting,此车无法行驶
			//3.因为行驶里程不够，只能继续等在路口
			if (car_original.idCurRoad != car.idCurRoad || car_original.location != car.location)//road发生变化或者location发生变化，视为转向成功
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
			addCar(cars[i]);
	}
}
