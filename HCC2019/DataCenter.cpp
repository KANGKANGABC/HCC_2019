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

void DataCenter::write_graph()
{
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
				if (cross[i].roadID_D != -1)
				{
					//先调度路口的D方向
					int idRoad = cross[i].roadID_D;
					if (road[idRoad - 5000].idTo == cross[i].id) //此cross为road的toID,
					{
						while ()//按顺序逐排调度该道路到达路口的全部车辆，终止条件为：road所有车不可行驶
						{

						}
						for (int j = 0; j < road[idRoad - 5000].channel; ++j)//遍历此road的所有FORWARD车道
						{
							if (road[idRoad - 5000].lane[j].laneCar.size() != 0 && road[idRoad - 5000].lane[j].laneCar[0].status == WAITTING)
								//如果该车道有车,且车辆为WAITTING状态，代表该车即将通过路口
							{
								switch (road[idRoad - 5000].lane[j].laneCar[0].dirCross)
								{
								NONE:
									break;
								DD:
									break;
								LEFT:
									break;
								RIGHT:
									break;
								default:
									break;
								}
								
							}
						}
					}
					else //此cross为road的fromID
					{
						for (int j = road[idRoad - 5000].channel; j < 2 * road[idRoad - 5000].channel; ++j)//遍历此road的所有BACKWARD车道
						{

						}
					}

					//
					


					road[idRoad - 5000].lane
				}
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