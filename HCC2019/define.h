#ifndef DEFINE_H_
#define DEFINE_H_

#include <string>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <vector>
#include <assert.h>
#include <deque>
#include <queue>
#include <map>
#include <stack>

#include "lib_io.h"

#define _DEBUG

#define MAX_ROAD_NUM    1000 //最大ROAD条数
#define MAX_CAR_NUM    90000 //最大CAR条数
#define MAX_CROSS_NUM    1000 //最大CROSS条数


#define INT_MAX 0x7fffffff
#define FLT_MAX 3.402823466e+38F

#define PARA_PERIOD 45

#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif

enum
{// 车道方向
	FORWARD,	// 正向
	BACKWARD,	// 逆向
};

enum
{// 该车辆是否在路口？如果在路口将要往哪边行驶？
	NONE, //不在路口
	DD,	// 直行 //不明白官方为什么用D字母表示直行
	LEFT,	// 左转
	RIGHT,  //右转
};

enum
{// 车辆运行状态 //请参考论坛中关于任务调度的解释
	SLEEPING,	// 等待出发（车库中）
	WAITTING,	// 等待行驶 
	FINESHED,   // 终止车辆 
};


#endif
