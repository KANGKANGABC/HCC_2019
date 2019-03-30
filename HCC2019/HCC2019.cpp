// HCC2019.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "define.h"
#include "lib_io.h"
#include "DataCenter.h"
#include "dijkstra.h"
#include "Scheduler.h"
#include "Algorithm.h"

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

	const char *answer_file = answerPath.c_str();

	char *data_road[MAX_ROAD_NUM];
	char *data_car[MAX_CAR_NUM];
	char *data_cross[MAX_CROSS_NUM];
	int road_line_num = read_file(data_road, MAX_ROAD_NUM, roadPath.c_str());
	int car_line_num = read_file(data_car, MAX_CAR_NUM, carPath.c_str());
	int cross_line_num = read_file(data_cross, MAX_CROSS_NUM, crossPath.c_str());

	DataCenter dc(data_road, road_line_num, data_car, car_line_num, data_cross, cross_line_num);
	dc.readCrossData();
	dc.readRoadData();
	dc.readCarData();


	Algorithm alg(dc);
	//Scheduler sd(dc);
	//sd.SchedulerTest();

	//alg.StaticAnalysis_SpeedBasic_AutoPara();
	//alg.unlockDead(78);
	//alg.DynamicPathByScheduler_SpeedBasic_AutoPara(1);

	//alg.ShortestTime_SpeedBasicRoadStatus_AutoPara(0);
	//alg.ShortestTime_SpeedBasic_AutoPara();

	alg.StaticAnalysisNor_SpeedBasicNoSame_AutoPara(122);

	dc.writeResult(answer_file);

	return 0;
}