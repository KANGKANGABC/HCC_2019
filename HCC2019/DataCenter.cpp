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


	road = new Road[m_road_num];//创建所有道路的对象
	cross = new Cross[m_cross_num];//创建所有路口的对象
	car = new Car[m_car_num];//创建所有汽车的对象
	carOrderTime.resize(m_car_num);
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
    
		car[i - 1].id = std::stoi(sp[0].substr(1));//去除左括号
		car[i - 1].idCrossFrom = std::stoi(sp[1]);
		car[i - 1].idCrossTo = std::stoi(sp[2]);
		car[i - 1].speed = std::stoi(sp[3]);
		car[i - 1].plantime = std::stoi(sp[4].substr(0, sp[4].size() - 1));//去除右括号
		car[i - 1].status = SLEEPING;//车的初始状态为SLEEPING
		car[i - 1].dirCross = NONE;//车的过路口状态为NONE

		vector<int>::iterator it;
		it = find(speedType.begin(), speedType.end(), car[i - 1].speed);
		if (it == speedType.end())
		{
			speedType.push_back(car[i - 1].speed);
		}
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
		line += std::to_string(car[i].starttimeAnswer);
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

void DataCenter::writeResultWithTime(const char *filename)
{
	result += "#(carId,StartTime,RoadId,Time,StartTime,DirMap...)\n";
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
		line += ", ";
		line += std::to_string(car[i].time);
		line += ", ";
		line += std::to_string(car[i].starttime);
		line += ", ";
		line += std::to_string(car[i].dirMap);
		line += ")\n";
		result += line;
	}
	const char *result_file = result.c_str();
	write_result(result_file, filename);
}


void DataCenter::swap(int i, int j)
{
	Car tmp;
	tmp = qCar[i];
	qCar[i] = qCar[j];
	qCar[j] = tmp;
}

void DataCenter::quicksort(int begin, int end)
{
	int i, j;
	i = begin + 1;
	j = end;
	if(begin < end)
	{
		while (i < j)
		{
			if (qCar[i].plantime > qCar[begin].plantime)
			{
				swap(i, j);
				j--;
			}
			else
				i++;
		}
		if (qCar[i].plantime > qCar[begin].plantime)
			i--;
		swap(i, begin);
		quicksort(begin, i - 1);
		quicksort(i + 1, end);
	}
}

void DataCenter::reorderCars()
{
	for (int i = 0; i < m_car_num; i++)
	{
		qCar.push_back(car[i]);//将id顺序的车辆放到qcar的vector中
	}

	int begin = 0;
	int end = qCar.size() - 1;
	quicksort(begin, end);

}
