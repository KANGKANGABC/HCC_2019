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
	}
	printf("readCrossData done!\n");
}

int DataCenter::calSysTime()
{
	//这里将carPathList路径列表初始化为1条路径
	std::vector<int> carPath { 1001, 1, 501, 502, 503, 516, 506, 505, 518, 508, 509, 524 };
	carPathList.push_back(carPath);

	//将carTask车辆调度任务列表初始化为1辆车
	std::vector<std::vector<int>> carTaskTmp;//避免和私有变量重名
	carTaskTmp.push_back({ 0, 1, 16, 6, 1, 0, 0, 0 });

	timeSysMachine = 0;//系统调度时间初始化为0

	while (carTaskTmp.size() != 0)
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

								//如果车到达路口，将车加入路口

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

		while ()//循环调度，直到所有的车辆行驶一个单位，也就是说所有车辆必须为FINESHED状态？
		{
			//按照升序调度所有路口

			//按照顺序调度所有道路

		}




		//第二步：处理所有路口等待的车辆
		for (int i = 0; i < carTaskTmp.size(); ++i)
		{
			if (carTaskTmp[i][7] != SLEEPING)
			{
				//让该车行驶,同时判断是否到终点
			}
		}

		//再调度等待上路行驶的车辆
		for (int i = 0; i < carTaskTmp.size(); ++i)
		{
			if (carTaskTmp[i][7] == SLEEPING)
			{
				if (carTaskTmp[i][4] == timeSysMachine)
				{
					//尝试将该车加入道路，并修改车的状态
					//如果加入道路失败，则修改该车启动时间
				}
			}
		}
		timeSysMachine ++;

	}



	return timeSysMachine;
}

