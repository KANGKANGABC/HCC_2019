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

	//std::string s = "config_10\\answer2.txt";//调试dijkstra用


	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;

	const char *answer_file = answerPath.c_str();
	//char *answer_file2 = &s[0]; //调试dijkstra用

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

	Scheduler sd(dc);

	sd.ReOrderStartBySpeed(39);
	//sd.ReOrderStartBySpeedAndStartCross(39);
	sd.reorderCars();//按照时间重排序车辆

	//测试路径拥堵探测函数
	/*
	bool b;
	std::vector<int > path;
	path.push_back(5011);
	path.push_back(5021);
	path.push_back(5031);
	std::map<string, float > mapstr;	//存储道路的拥挤程度
	cout << mapstr.size() << endl;
	sd.mapUpdate(mapstr, 5031, 0.8);
	cout << mapstr.size() << endl;
	cout << mapstr[to_string(5031)] << endl;
	b = sd.judgement(mapstr, path);
	cout << b << endl;
	*/

	int time = 0;
	int para = 0;
  
	//sd.getPath();
	//sd.getPathWeightOne();

	//sd.ReOrderStartByTime(PARA_PERIOD);
	//sd.getPathByTime_reorderCars();//获得车辆的路径信息
	//sd.getTimeByDir(90);
	//sd.getStartTime(470);
	//dc.writeResultWithTime(answer_file);
	//sd.ReOrderStartByTime(PARA_PERIOD);
	//dc.writeResult(answgetParaBySchedulerer_file);

	//para = sd.getParaByScheduler();
	//time = sd.getPathByScheduler(9);
	time = sd.unlockDead(80);
	//sd.getPathByTime();//获得车辆的路径信息
	//sd.reorderCars();//按照时间重排序车辆
	//sd.getStartTime_loadbalance(550);

	//dc.writeResult(answer_file);
	//sd.getPathByTime();//获得车辆的路径信息
	//dc.writeResult(answer_file);
	//sd.getPathByScheduler();
	//sd.getPathByScheduler();
	//sd.getPathByTime_dynamic();//获得车辆的路径信息
	//int time = sd.getSysTime();
	//sd.getPathByTime_reorderCars();//获得车辆的路径信息
	
	//dc.writeResultWithTime(answer_file);

	sd.getPathByTime_dynamic();//获得车辆的路径信息
	int time = sd.getSysTime();
	dc.writeResult(answer_file);
	PRINT("time:%d\n",time);

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
