#ifndef DATA_CENTER_H_
#define DATA_CENTER_H_

#include "lib_io.h"
#include "define.h"
#include "Road.h"

class TimeGraph
{
public:
	std ::vector < std :: vector< float> >timeGraph;  //时间邻接矩阵
};

struct Dis {
	std::vector<int> path;
	float value;
	bool visit;
	Dis() {
		visit = false;	//判断是否已经被访问
		value = 0;		//路径的长度
	}
};

class DataCenter
{
public:
	DataCenter();
	DataCenter(char *data_road[MAX_ROAD_NUM], int road_count, char *data_car[MAX_CAR_NUM], int car_count, char *data_cross[MAX_CROSS_NUM], int cross_count);
	~DataCenter();

	//读入数据
	void readRoadData();
	void readCarData();
	void readCrossData();

	//将邻接矩阵写出到文件
	void write_graph();

	//统计car.txt中的各车辆速度类型到Vector speedType中
	void getCarSpeedType ();

	//计算输入速度下时间邻接矩阵
	void getTimeGraph (int order, int speed );

	//计算得到所有速度下的时间邻接矩阵
	void getAllTimeGraph();

	//Dijkstra算法，输入点begin，输出点begin到各点的最短时间
	std :: vector<int>  Dijkstra(int begin ,int end ,int speed );
	//输出begin到其余点的最短时间路径信息
	void print_path(int begin);

	//计算当前路径的运行时间
	int calSysTime();

	//判断某cross的某road是否存在需要直行的车
	bool isBeDD(int idRoad,int idCross);

	//判断某cross的某road是否存在需要左转的车
	bool isBeLEFT(int idRoad, int idCross);

	//判断某cross的某road是否可以行驶进入
	bool isCanEnter(int idRoad, int idCross);

	//驱动某car行驶
	void carRun(Car car);

	enum
	{// 车辆运行状态 //请参考论坛中关于任务调度的解释
		SLEEPING,	// 等待出发（车库中）
		WAITTING,	// 等待行驶 
		FINESHED,   // 终止车辆 
	};

	//所有道路
	//道路对象的指针
	Road * road;

	//所有路口的指针
	Cross * cross;

private:

	char **inputRoadData;//输入道路数据
	char **inputCarData;//输入道路数据
	char **inputCrossData;//输入道路数据

	int m_road_num;//ROAD数量
	int m_car_num;//CAR数量
	int m_cross_num;//CROSS数量

	int car_speed_num; //car 的速度种类在getCarSpeedType()中被赋值

	//道路有向图邻接矩阵
	std::vector<std::vector<int> > graphRoad;	//距离邻接矩阵，不邻接的点用正无穷表示
	std::vector<std::vector<int> > graphMaxSpeed;	//道路最大速度邻接矩阵，不邻接的点用0表示
	//std::vector<std::vector<float> > timeGraph;		// 时间邻接矩阵，不邻接的点用正无穷表示

	//存储车辆的速度种类的向量
	std::vector<int> speedType;

	//存储速度种类个时间邻接矩阵的指针
	TimeGraph * timeGraphPoint;

	//Dijkstra算法中记录各个顶点的最短路径信息
	//Dijkstra算法里用到的结构体

	Dis * dis;


	//路口信息表
	//(id,roadId,roadId,roadId,roadId)
	std::vector<std::vector<int> > crossList;

	//Car调度任务表
	std::vector<std::vector<int> > carTask;

	//路径列表
	std::vector<std::vector<int> > carPathList;

	//系统时间
	int timeSysMachine;

	
};


#endif