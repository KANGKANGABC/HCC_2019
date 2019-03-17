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


	//为dis申请空间
	dis = new DisFloat[m_cross_num];

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
	delete[] this->dis;
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
		car[i - 1].starttime = carTask[i - 1][4] + (8 - car[i - 1].speed )*80 + i % 100;//这里给自己挖了一个坑
		car[i - 1].status = SLEEPING;//车的初始状态为SLEEPING
		car[i - 1].dirCross = NONE;//车的过路口状态为NONE

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
		cross[i - 1].roadID_T = std::stoi(sp[1]);
		cross[i - 1].roadID_R = std::stoi(sp[2]);
		cross[i - 1].roadID_D = std::stoi(sp[3]);
		cross[i - 1].roadID_L = std::stoi(sp[4].substr(0, sp[4].size() - 1));//去除右括号
		cross[i - 1].roadID.resize(4);
		cross[i - 1].roadID = { cross[i - 1].roadID_T ,cross[i - 1].roadID_R ,cross[i - 1].roadID_D ,cross[i - 1].roadID_L };
	}
	printf("readCrossData done!\n");
}

/******************************统计car.txt中的各车辆速度数量到m_cross_num;，类型到Vector speedType中********此处有常数3！！是车辆速度种类****************/
void DataCenter::getCarSpeedType()
{
	printf("getCarSpeedType\n");
	//统计车辆速度数量
	std :: vector<int> carSpeed;
	for (int i = 0; i < m_car_num ; ++i)
	{
		carSpeed.push_back(carTask[i][3]);		//将carTask中的速度一列存到carspeed向量中
	}
	sort(carSpeed.begin(),carSpeed.end());           //将速度排序

	car_speed_num = 1;
	speedType.push_back(carSpeed[0]);
	int speed = carSpeed[0];
	for (int i = 0; i < m_car_num; ++i)
	{
		if (speed != carSpeed[i])
		{
			car_speed_num++;
			speed = carSpeed[i];
			speedType.push_back(carSpeed[i]);

		}
	}
	printf("getCarSpeedType done!\n");
}

/***************根据输入的车辆速度计算时间邻接矩阵的getTimeGraph（） 和得到所有速度下时间邻接矩阵的getAllTimeGraph（）******************************************/
void DataCenter::getTimeGraph(int order,int speed)
{
	printf("getTimeGraph\n");
	//根据输入速度和道路限速邻接矩阵，得到车辆行驶过程中最大速度邻接矩阵
	for(int i=0; i< m_cross_num; i++)
		for (int j = 0; j < m_cross_num; j++)
		{
			graphMaxSpeed[i][j] = std:: min(graphMaxSpeed[i][j] , speed);
		}

	//根据距离邻接矩阵和车辆行驶最大速度邻接矩阵得到时间邻接矩阵
	for (int i = 0; i < m_cross_num; i++)
		for (int j = 0; j < m_cross_num; j++)
		{
			if (graphRoad[i][j] != INT_MAX)
			{
				timeGraphPoint[order]. timeGraph[i][j] = graphRoad[i][j] / float(graphMaxSpeed[i][j]); 
			}
		}
	printf("getTimeGraph done!\n");
}

void DataCenter::getAllTimeGraph()
{
	printf("getAllTimeGraph\n");

	//声明存储不同速度邻接矩阵的容量
	//存储速度种类个时间邻接矩阵的指针
	timeGraphPoint = new TimeGraph[car_speed_num];

	//将时间邻接矩阵大小设置为36，不邻接的点初值设为正无穷
	for (int i = 0; i < car_speed_num; i++)
	{
		timeGraphPoint[i].timeGraph.resize(m_cross_num);
		for (int j = 0; j < m_cross_num; ++j)
		{
			timeGraphPoint[i].timeGraph[j].resize(m_cross_num);
		}
	}
	for (int i = 0; i < car_speed_num; i++)
		for (int j = 0; j < m_cross_num; ++j)
			for (int k = 0; k < m_cross_num; ++k)
			{
				timeGraphPoint[i].timeGraph[j][k] = FLT_MAX;
			}

//调用getTimegraph矩阵
	for (int i = 0; i < car_speed_num; i++)
		getTimeGraph( i, speedType[i]);
	printf("getAllTimeGraph done!\n");
}

/************************************* Dijkstra 算法  和输出路径的函数 print_path(int begin)*****************************************************/

std :: vector<int> DataCenter::Dijkstra(int begin, int end, int speed) {
	dis = new DisFloat[m_cross_num];
	//根据speed确定调用时间矩阵的序号
	int order = 0;  //表示调用timeGraphPoint的下标
	for (int i = 0; i < car_speed_num; i++)
	{
		if (speedType[i] == speed)
			order = i;
	}

	//初始化dis数组
	int i;
	for (i = 0; i < m_cross_num; i++) {
		//设置当前的路径
		dis[i].path.push_back(begin);
		dis[i].path.push_back( i+1 );
		dis[i].value = timeGraphPoint[order].timeGraph[begin - 1][i];	//将邻接数组起点的那一行的值赋给dis数组
	}

	//设置起点到起点自己的路径为0
	dis[begin - 1].value = 0;
	dis[begin - 1].visit = true;

	int count = 1;
	//计算到其他各顶点的最短路径
	while (count != m_cross_num) {
		//temp用于保存当前dis数组中最小的那个下标
		//min记录当前的最小值
		int temp = 0;
		float min = FLT_MAX;
		//这里的for循环我的理解就是算法中优先队列的作用（找目前最短的点），选择准备进行松弛的点，给下一步的for循环进行松弛操作
		for (i = 0; i < m_cross_num; i++) {
			if (!dis[i].visit && dis[i].value < min) {
				min = dis[i].value;
				temp = i;
			}
		}

		dis[temp].visit = true;		//把上一步找到的准备进行松弛操作的点加入已找到的最短路径集合（实际上就是下次不再入优先队列，每个点至进行一次入队）
		++count;
		//下面这个for循环这么理解（更新的是temp指向的点的值），它是将所有的点都进行了一次松弛更新的操作，如果满足条件则更新否则不更新，在算法中写的是值操作相邻的点进行操作，因为不相邻的没有意义（这里就用无穷达来表达了这种情况！因此它直接所有点遍历，如果可以只存邻接的点那会更好！）
		for (i = 0; i < m_cross_num; i++) {
			if (!dis[i].visit && timeGraphPoint[order].timeGraph[temp][i] != FLT_MAX && (dis[temp].value + timeGraphPoint[order].timeGraph[temp][i]) < dis[i].value)
			{
				dis[i].value = dis[temp].value + timeGraphPoint[order].timeGraph[temp][i];
				dis[i].path.assign (dis[temp].path.begin(), dis[temp].path.end());
				dis[i].path.push_back(i + 1);
			}
		}
	}
	
	return dis[end - 1].path;

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

//获取邻接矩阵
std::vector<std::vector<int> > DataCenter::getArc()
{
	return graphRoad;
}

void DataCenter::writeResult(char *filename)
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
	graph.createGraph(graphRoad);

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

void DataCenter::getPathBytime()
{
	getCarSpeedType();
	getAllTimeGraph();
	for (int i = 0; i < m_car_num; ++i)
	{
		vector<int> pathCross = Dijkstra(car[i].idCrossFrom, car[i].idCrossTo, car[i].speed);
		vector<int> pathRoad(pathCross.size() - 1);
		for (int j = 0; j < pathRoad.size(); ++j)
		{
			pathRoad[j] = graphC2R[pathCross[j] - 1][pathCross[j + 1] - 1];
			//assert(pathRoad[j] != 0);
		}
		car[i].path = pathRoad;
	}
}

void DataCenter::saveCarsBySpeed()
{
	getCarSpeedType();
	carsBySpeed.resize(car_speed_num);
	Graph_DG graph(vexnum, edge);
	graph.createGraph(graphRoad);
	for (int i = 0; i < m_car_num; ++i)
	{
		for (int j = 0; j < car_speed_num; ++j)
		{
			if (car[i].speed == speedType[j])
			{
				vector<int> pathCross = graph.Dijkstra(car[i].idCrossFrom, car[i].idCrossTo);
				vector<int> pathRoad(pathCross.size() - 1);
				for (int m = 0; m < pathRoad.size(); ++m)
				{
					pathRoad[m] = graphC2R[pathCross[m] - 1][pathCross[m + 1] - 1];
				}
				car[i].path = pathRoad;
				carsBySpeed[j].push_back(car[i]);
			}
		}
	}
}

