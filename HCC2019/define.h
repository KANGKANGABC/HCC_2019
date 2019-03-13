#ifndef DEFINE_H_
#define DEFINE_H_

#include <string>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <vector>

#define MAX_ROAD_NUM    100 //最大ROAD条数
#define MAX_CAR_NUM    150 //最大CAR条数
#define MAX_CROSS_NUM    100 //最大CROSS条数

enum
{// 车道方向
	FORWARD,	// 正向
	BACKWARD,	// 逆向
};

enum
{// 车辆在路口的状态
	NONE,	// 不准备通过路口
	DD,	// 直行
	LEFT, //左转
	RIGHT, //右转
};

#endif