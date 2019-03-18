// HCC2019.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "define.h"
#include "lib_io.h"
#include "DataCenter.h"
#include "dijkstra.h"
#include "Scheduler.h"

using namespace std;

int main(int argc, char *argv[])
{
	std::cout << "Begin" << std::endl;

	if (argc < 5) {
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}

	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string answerPath(argv[4]);


	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;

	char *answer_file = argv[4];

	char *data_road[MAX_ROAD_NUM];
	char *data_car[MAX_CAR_NUM];
	char *data_cross[MAX_CROSS_NUM];
	int road_line_num = read_file(data_road, MAX_ROAD_NUM, roadPath.c_str());
	int car_line_num = read_file(data_car, MAX_CAR_NUM, carPath.c_str());
	int cross_line_num = read_file(data_cross, MAX_CROSS_NUM, crossPath.c_str());

	DataCenter dc(data_road, road_line_num, data_car, car_line_num, data_cross, cross_line_num);
	dc.readRoadData();
	dc.readCarData();
	dc.readCrossData();

	//测试dijkstra算法
	int vexnum, edge;
	std::vector<std::vector<int> > tmp1 = dc.getRoadvArc(); //得到道路限速邻接矩阵
	std::vector<std::vector<int> > tmp = dc.getArc(); //得到路长邻接矩阵
	vexnum = dc.getCrossNum();
	edge = dc.getRoadNum();

	Graph_DG graph(vexnum, edge);
	graph.createArcGraph(tmp);
	graph.createArcRoadvGraph(tmp1);
	vector<int> path = graph.Dijkstra(16, 34, 3);
	graph.print();
	graph.printRoadv();
	graph.printTimeArc();

	cout << "最短路径为";
	for (int i = 0; i < path.size(); i++)
		cout << path.at(i) << " ";
	cout << endl;
	//dc.getPathBytime();
	//dc.writeResult(answer_file);


	//Scheduler sd(dc);
	//sd.getPath();//获得车辆的路径信息
	//int time = sd.getSysTime();

	// TODO:read input filebuf
	// TODO:process
	// TODO:write output file

	return 0;
}

// 运行程序: Ctrl + F5 或调试 >“开始执行(不调试)”菜单
// 调试程序: F5 或调试 >“开始调试”菜单

// 入门提示: 
//   1. 使用解决方案资源管理器窗口添加/管理文件
//   2. 使用团队资源管理器窗口连接到源代码管理
//   3. 使用输出窗口查看生成输出和其他消息
//   4. 使用错误列表窗口查看错误
//   5. 转到“项目”>“添加新项”以创建新的代码文件，或转到“项目”>“添加现有项”以将现有代码文件添加到项目
//   6. 将来，若要再次打开此项目，请转到“文件”>“打开”>“项目”并选择 .sln 文件
