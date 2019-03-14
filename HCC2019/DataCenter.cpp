#include "DataCenter.h"
#include "Tools.h"
#include <iostream>
#include <algorithm>
#include <string>


DataCenter::DataCenter()
{
}

DataCenter::DataCenter(char *data_road[MAX_ROAD_NUM],int road_count, char *data_car[MAX_CAR_NUM], int car_count, char *data_cross[MAX_CROSS_NUM], int cross_count)
{
	inputRoadData = data_road;
	m_road_num = road_count - 1;//忽略第一行注释
	inputCarData = data_car;
	m_car_num = car_count - 1;//忽略第一行注释
	inputCrossData = data_cross;
	m_cross_num = cross_count - 1;//忽略第一行注释

	//将邻接矩阵大小设置为36
	graphRoad.resize(36);
	for (int i = 0; i < 36; ++i) {
		graphRoad[i].resize(36);
	}
	//将邻接矩阵初始化为正无穷
	for (int i= 0; i < 36; ++i)
		for (int j = 0; j < 36; ++j)
		{
			graphRoad[i][j] = INT_MAX;
		}

	//将车辆类型向量大小设置为3
	speedType.resize(3);
	//将车辆类型向量初始化为0
	for (int i = 0; i < 3; i++)
		speedType[i] = 0;

	//Car调度任务向量大小设置
	//Car任务数量为所有需要调度的Car数
	carTask.resize(m_car_num);
	// |   0     1    2     3       4        5           6         7   |
	// | 车辆ID 起点 终点 车辆速度 出发时间 当前道路ID 当前道路位置 当前状态|
	for (int i = 0; i < m_car_num; ++i) {
		carTask[i].resize(8);
	}

	//路口信息表大小设置
	crossList.resize(m_cross_num);
	for (int i = 0; i < m_cross_num; ++i) {
		crossList[i].resize(5);
	}

	road = new Road[m_road_num];//创建所有道路的对象
	cross = new Cross[m_cross_num];//创建所有路口的对象
}

DataCenter::~DataCenter()
{
	delete[] this->road;
}

/******************************************************** readRoadData() ,readCarData() ,readCrossData() ******************************************************************/
void DataCenter::readRoadData()
{
	printf("readRoadData\n");
	for (int i = 1; i <= m_road_num; ++i)//忽略第0行数据
	{
		std::string roadInfo = inputRoadData[i];
		std::vector<std::string> sp = Tools::split(roadInfo, ", ");

		this->road[i - 1].id = std::stoi(sp[0].substr(1));//去除左括号
		this->road[i - 1].length = std::stoi(sp[1]);
		this->road[i - 1].speed = std::stoi(sp[2]);
		this->road[i - 1].channel = std::stoi(sp[3]);
		this->road[i - 1].idFrom = std::stoi(sp[4]);
		this->road[i - 1].idTo = std::stoi(sp[5]);
		this->road[i - 1].isDuplex = std::stoi(sp[6].substr(0, sp[6].size() - 1));//去除右括号

		//初始化每个Road中Lane
		this->road[i - 1].CreateLane();

		if (sp[6].substr(0, 1) == "1")
		{
			graphRoad[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[1]);
			graphRoad[std::stoi(sp[5]) - 1][std::stoi(sp[4]) - 1] = std::stoi(sp[1]);
		}
		else
		{
			graphRoad[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[1]);
		}

	}
	printf("readRoadData done!\n");
}


void DataCenter::readCarData()
{
	printf("readCarData\n");
	// |   0     1    2     3       4        5           6         7   |
    // | 车辆ID 起点 终点 车辆速度 出发时间 当前道路ID 当前道路位置 当前状态|
	for (int i = 1; i <= m_car_num; ++i)//忽略第0行数据
	{
		std::string carInfo = inputCarData[i];
		std::vector<std::string> sp = Tools::split(carInfo, ", ");
		carTask[i - 1][0] = std::stoi(sp[0].substr(1));//去除左括号
		carTask[i - 1][1] = std::stoi(sp[1]);
		carTask[i - 1][2] = std::stoi(sp[2]);
		carTask[i - 1][3] = std::stoi(sp[3]);
		carTask[i - 1][4] = std::stoi(sp[4].substr(0, sp[4].size()-1));//去除右括号
		carTask[i - 1][5] = 0;
		carTask[i - 1][6] = 0;
		carTask[i - 1][7] = SLEEPING;
	}
	printf("readCarData done!\n");
}

void DataCenter::readCrossData()
{
	printf("readCrossData\n");
	for (int i = 1; i <= m_cross_num; ++i)//忽略第0行数据
	{
		std::string crossInfo = inputCrossData[i];
		std::vector<std::string> sp = Tools::split(crossInfo, ", ");
		crossList[i - 1][0] = std::stoi(sp[0].substr(1));//去除左括号
		crossList[i - 1][1] = std::stoi(sp[1]);
		crossList[i - 1][2] = std::stoi(sp[2]);
		crossList[i - 1][3] = std::stoi(sp[3]);
		crossList[i - 1][4] = std::stoi(sp[4].substr(0, sp[4].size() - 1));//去除右括号

		
		cross[i - 1].id = std::stoi(sp[0].substr(1));//去除左括号
		cross[i - 1].roadID_D = std::stoi(sp[1]);
		cross[i - 1].roadID_L = std::stoi(sp[2]);
		cross[i - 1].roadID_R = std::stoi(sp[3]);
		cross[i - 1].roadID_T = std::stoi(sp[4].substr(0, sp[4].size() - 1));//去除右括号
	}
	printf("readCrossData done!\n");
}

/************************************************************************ 输出邻接矩阵到road_graph.txt *************************************************************************/
void DataCenter::write_graph()
{
	printf("write_graph\n");

	int rowCount = 0;
	int columnCount = 0;
	std::string graph;

	for (rowCount = 0; rowCount < 36; ++rowCount)
	{
		for (columnCount = 0; columnCount < 36; ++columnCount)
		{
			graph += std::to_string(graphRoad[rowCount][columnCount]);
			graph += " ";
		}
		graph += "\n";
	}

	const char *graph_file = graph.c_str();
	const char * fileName = "road_graph.txt";

	write_result(graph_file, fileName);

	printf("write_graph done!\n");
}

/*******************************统计car.txt中的各车辆速度类型到Vector speedType中********此处有常数3！！是车辆速度种类****************/
void DataCenter::getCarSpeedType()
{
	printf("getCarSpeedType\n");

	int  typeNum = 0;		                                             //已有速度种类

	for (int i = 0; i < m_car_num - 1; ++i)
	{
		bool haveThisSpeed = false;

		for (int j = 0; j < 3; ++j)
		{
			if (carTask[i][3] == speedType[j])
				haveThisSpeed = true;
		}

		if (haveThisSpeed == false)
		{
				speedType[typeNum++] = carTask[i][3];
		}

		if (typeNum == 3)                                             //该样例中速度只有三种，集齐三种速度就能退出了
			break;         
	}

	printf("getCarSpeedType done!\n");
}

/******************************************************************************为调度部分代码*****************************************************************************************/
int DataCenter::calSysTime()
{
	//新建一个car对象，对系统进行测试
	Car *car = new Car;
	car->id = 10000;
	car->location = 0;
	car->speed = 6;
	car->status = SLEEPING;
	car->path = { 5029, 5040, 5051, 5057, 5058 };//规划一个简单路径

	timeSysMachine = 0;//系统调度时间初始化为0

	while (1)//终止条件为所有车辆调度完成
	{
		//先调度在路上行驶的车辆
		//第一步：先处理所有道路上的车辆，进行遍历扫描
		for (int i = 0; i < m_road_num; ++i)//按道路ID升序进行调度
		{
			for (int j = 0; j < road[i].channel * (1 + road[i].isDuplex); ++j)
			{
				//先从正向开始
				if (road[i].lane[j].laneCar.size() > 0)//该车道有车
				{
					for (int m = 0; m < road[i].lane[j].laneCar.size(); ++m)//遍历该车道的所有车辆
					{
						//判断该车前面有没有车
						if (m == 0)//m==0 代表该车为该车道第一辆车
						{
							//该车行驶后是否还在相同路径上？
							if (road[i].lane[j].laneCar[m].location + std::min(road[i].lane[j].laneCar[m].speed, road[i].speed) <= road[i].length)
							{
								road[i].lane[j].laneCar[m].location += std::min(road[i].lane[j].laneCar[m].speed, road[i].speed);
								road[i].lane[j].laneCar[m].status = FINESHED;//该车行驶完成
							}
							else//如果不在该路径，那么该车设置为等待状态
							{
								road[i].lane[j].laneCar[m].status = WAITTING;//该车等待驶出路口
								//如果该路口为车的终点
								//那么此车调度完成
							}
						}
						else
						{
							//判断能否完成行驶，前面的车是否形成阻挡？
							if (road[i].lane[j].laneCar[m].location + std::min(road[i].lane[j].laneCar[m].speed, road[i].speed) < road[i].lane[j].laneCar[m - 1].location)
							{
								//前面的车不行成阻挡
								road[i].lane[j].laneCar[m].location += std::min(road[i].lane[j].laneCar[m].speed, road[i].speed);
								road[i].lane[j].laneCar[m].status = FINESHED;//该车行驶完成
							}
							else
							{
								//前面的车形成阻挡,则需要根据前面阻挡车的状态来决定
								if (road[i].lane[j].laneCar[m - 1].status == FINESHED)//如果前车行驶完成，则行驶至前车后一位置
								{
									road[i].lane[j].laneCar[m].location = road[i].lane[j].laneCar[m - 1].location - 1;//行驶至前面车后
									road[i].lane[j].laneCar[m].status = FINESHED;//该车行驶完成
								}
								else if(road[i].lane[j].laneCar[m - 1].status == WAITTING)//如果前车等待行驶，则此车也等待行驶
								{
									road[i].lane[j].laneCar[m].status = WAITTING;//该车等待
								}
								
							}
						}
					}
				}
			}
		}

		//第二步：处理所有路口等待的车辆
		
		while (1)//循环调度，直到所有的车辆行驶一个单位，也就是说所有车辆必须为FINESHED状态？对的，论坛写到了这一点
		{
			//按照升序调度所有路口
			for (int i = 0; i < m_cross_num; ++i)
			{
				int idCross = cross[i].id;//获得路口ID
				while (1)//循环调度路口四个方向的车，直到全部车辆完成调度，或者阻塞
				{
					bool isWorkingCross = false;//标志变量，如果一个循环后没有任何一辆车被调度，则退出循环
					if (cross[i].roadID_T != -1)//先调度直行路口
					{
						int idRoad = cross[i].roadID_T;//被调度的道路id
						int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
						if (road[idRoad - 5000].idTo == cross[i].id)//如果cross为道路的出方向
							idStartLane = road[idRoad - 5000].channel;
						while (1)
						{
							bool isWorkingRoad = false;
							for (int j = idStartLane; j < idStartLane + road[idRoad - 5000].channel; ++j)//遍历所有lane
							{
								if (road[idRoad - 5000].lane[j].laneCar.size() != 0)//lane非空则调度
								{
									switch (road[idRoad - 5000].lane[j].laneCar[0].dirCross)
									{
									NONE:
										break;
									DD://直行>左转>右转
										//判断转入的road是否可以行驶

										break;
									LEFT://左转>右转
										//判断即将转入的方向是否有直行进入的车辆
										if (!isBeDD(cross[i].roadID_L, i))
										{
											//判断转入的road是否可以行驶
										}
										break;
									RIGHT://右转优先级最低
										//判断即将转入的方向是否有直行进入的车辆
										if (!isBeDD(cross[i].roadID_R, i))
										{
											//判断即将转入的方向是否有左转进入的车辆
											if (!isBeLEFT(cross[i].roadID_D, i))
											{
												//判断转入的road是否可以行驶

											}
										}
										break;
									default:
										break;
									}
								}
							}
							if (!isWorkingRoad)//如果本轮调度未调度任何车辆，则退出调度循环
								break;
						}

					}


					if (!isWorkingCross)//如果一个循环后没有任何一辆车被调度，则退出调度循环
						break;
				}

				//
				//路口ID
				;
				//根据cross的顺序，遍历road，再遍历road的lane，调度在路口WAITTING的车（每次路口调度，只调度路口的一辆车）
			}




			//按照顺序调度所有道路
			//再调度道路中WAITTING的车
		}
		
		timeSysMachine ++;

	}



	return timeSysMachine;
}

bool DataCenter::isBeDD(int idRoad, int idCross)
{
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (road[idRoad - 5000].idTo == cross[idCross].id)//如果cross为道路的出方向
		idStartLane = road[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + road[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (road[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (road[idRoad - 5000].lane[j].laneCar[0].dirCross == DD)
				return true;//存在直行车辆
		}
	}

	return false;
}

bool DataCenter::isBeLEFT(int idRoad, int idCross)
{
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (road[idRoad - 5000].idTo == cross[idCross].id)//如果cross为道路的出方向
		idStartLane = road[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + road[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (road[idRoad - 5000].lane[j].laneCar.size() != 0)
		{
			if (road[idRoad - 5000].lane[j].laneCar[0].dirCross == LEFT)
				return true;//存在左转车辆
		}
	}
	return false;
}

bool DataCenter::isCanEnter(int idRoad, int idCross)
{
	int idStartLane = 0;//如果cross为道路的出方向，需要调度 0 1 2车道，否则调度 3 4 5车道
	if (road[idRoad - 5000].idFrom == cross[idCross].id)//如果cross为道路的入方向
		idStartLane = road[idRoad - 5000].channel;
	for (int j = idStartLane; j < idStartLane + road[idRoad - 5000].channel; ++j)//遍历所有lane
	{
		if (road[idRoad - 5000].lane[j].laneCar.size() < road[idRoad - 5000].length)
		{
			//将车辆加入新的road
			//

			return true;//存在空位，可加入
		}
	}
	return false;
}

void DataCenter::carRun(Car car)
{
	//预设车的路径已经规划好，行驶过程中会更新path，path为将要进入的道路，如果进入下一条道路，
	//将上一条道路从path中删除，便于编程（不用每次都查path自己接下来走那条路径）

	//如果车处于SLEEP状态，将其加入对应道路
		//如果加入道路失败，将Car的起始时间加1，等待下次调度

	//如果车处于WAITTING/FINSHED状态
		//判断该车是否在路口，如果不是在路口等待，那么正常行驶（WAITTING/FINSHED）
		//判断该车是否在路口，如果已经在路口等待，那么将该车驶出到下一道路
}
