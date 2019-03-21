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

	//将速度邻接矩阵大小设置为36，不邻接的点初值为0
	graphMaxSpeed.resize(m_cross_num);
	for (int i = 0; i < m_cross_num; ++i)
	{
		graphMaxSpeed[i].resize(m_cross_num);
	}
  
	vexnum = getCrossNum();
	edge = getRoadNum();

	//将邻接矩阵大小设置为36*36
	graphRoad.resize(m_cross_num);
	for (int i = 0; i < m_cross_num; ++i) {
		graphRoad[i].resize(m_cross_num);
	}

	for (int i = 0; i < m_cross_num; ++i)
	{
		for (int j = 0; j < m_cross_num; ++j)
		{
			graphRoad[i][j] = INT_MAX;
		}
	}

	//将graphC2R大小设置为36*36
	graphC2R.resize(m_cross_num);
	for (int i = 0; i < m_cross_num; ++i) {
		graphC2R[i].resize(m_cross_num);
	}

	//将graphRoadStatusByDS大小设置为路口的数量
	graphRoadStatusByDS.resize(m_cross_num);
	for (int i = 0; i < m_cross_num; ++i) {
		graphRoadStatusByDS[i].resize(m_cross_num);
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
	car = new Car[m_car_num];//创建所有汽车的对象
}

DataCenter::~DataCenter()
{
	delete[] this->road;
	delete[] this->cross;
	delete[] this->car;
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

			graphMaxSpeed[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[2]);
			graphMaxSpeed[std::stoi(sp[5]) - 1][std::stoi(sp[4]) - 1] = std::stoi(sp[2]);
      
			graphC2R[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = this->road[i - 1].id;
			graphC2R[std::stoi(sp[5]) - 1][std::stoi(sp[4]) - 1] = this->road[i - 1].id;
		}
		else
		{
			graphRoad[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[1]);
			graphMaxSpeed[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = std::stoi(sp[2]);
			graphC2R[std::stoi(sp[4]) - 1][std::stoi(sp[5]) - 1] = this->road[i - 1].id;
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
		carTask[i - 1][4] = std::stoi(sp[4].substr(0, sp[4].size() - 1));//去除右括号
		carTask[i - 1][5] = 0;
		carTask[i - 1][6] = 0;
		carTask[i - 1][7] = SLEEPING;

		car[i - 1].id = carTask[i - 1][0];
		car[i - 1].idCrossFrom = carTask[i - 1][1];
		car[i - 1].idCrossTo = carTask[i - 1][2];
		car[i - 1].speed = carTask[i - 1][3];
		car[i - 1].plantime = carTask[i - 1][4];
		//car[i - 1].starttime = carTask[i - 1][4] + i%400;//这里给自己挖了一个坑
		int n2, n4, n6, n8;
		n2 = 40;
		n4 = 40;
		n6 = 40;
		n8 = 40;
		switch (car[i - 1].speed)
		{
		case 2:
			car[i - 1].plantime = car[i - 1].plantime + 6 * n2 + i % (2 * n2 - 10);
			break;
		case 4:
			car[i - 1].plantime = car[i - 1].plantime + 4 * n4 + i % (2 * n4 - 10);
			break;
		case 6:
			car[i - 1].plantime = car[i - 1].plantime + 2 * n6 + i % (2 * n6 - 10);
			break;
		case 8:
			car[i - 1].plantime = car[i - 1].plantime + 0 * n8 + i % (2 * n8 - 10);
			break;
		default:
			break;
		}
		car[i - 1].starttime = car[i - 1].plantime;
		//car[i - 1].plantime = carTask[i - 1][4] + (8 - car[i - 1].speed) * n + i % (2*n);//这里给自己挖了一个坑
		//car[i - 1].starttime = carTask[i - 1][4] + (8 - car[i - 1].speed )*80 + i % 160;//这里给自己挖了一个坑
		//car[i - 1].starttime = carTask[i - 1][4];

		car[i - 1].status = SLEEPING;//车的初始状态为SLEEPING
		car[i - 1].dirCross = NONE;//车的过路口状态为NONE
	}
	sort(speedType.begin(), speedType.end());
	car_speed_num = speedType.size();
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
		cross[i - 1].roadID_T = std::stoi(sp[1]);
		cross[i - 1].roadID_R = std::stoi(sp[2]);
		cross[i - 1].roadID_D = std::stoi(sp[3]);
		cross[i - 1].roadID_L = std::stoi(sp[4].substr(0, sp[4].size() - 1));//去除右括号
		cross[i - 1].roadID.resize(4);
		cross[i - 1].roadID = { cross[i - 1].roadID_T ,cross[i - 1].roadID_R ,cross[i - 1].roadID_D ,cross[i - 1].roadID_L };
	}
	printf("readCrossData done!\n");
}

//获取点和边的数量
int DataCenter::getRoadNum()
{
	return m_road_num;
}

int DataCenter::getCrossNum()
{
	return m_cross_num;
}

//获取路长邻接矩阵
std::vector<std::vector<int> > DataCenter::getArc()
{
	return graphRoad;
}

//获取道路限速邻接矩阵
std::vector<std::vector<int> > DataCenter::getRoadvArc()
{
	return graphMaxSpeed;
}

void DataCenter::writeResult(const char *filename)
{
	result += "#(carId,StartTime,RoadId...)\n";
	for (int i = 0; i < m_car_num; ++i)
	{
		std::string line = "(" + std::to_string(car[i].id);
		line += ", ";
		line += std::to_string(car[i].starttime);
		for (int j = 0; j < car[i].path.size(); ++j)
		{
			line += ", ";
			line += std::to_string(car[i].path[j]);
		}
		line += ")\n";
		result += line;
	}
	const char *result_file = result.c_str();
	write_result(result_file, filename);
}

void DataCenter::getPath()
{
	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(graphRoad);

	for (int i = 0; i < m_car_num; ++i)
	{
		vector<int> pathCross = graph.Dijkstra(car[i].idCrossFrom, car[i].idCrossTo);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		car[i].path = pathRoad;
	}
}