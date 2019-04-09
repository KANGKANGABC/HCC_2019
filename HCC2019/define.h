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
#include <cstdlib>

#include "lib_io.h"

#define _DEBUG

#define MAX_ROAD_NUM    1000 //���ROAD����
#define MAX_CAR_NUM    90000 //���CAR����
#define MAX_CROSS_NUM    1000 //���CROSS����
#define MAX_PATHPRESET_NUM    50000 //���PATHPRESET����


#define INT_MAX 0x7fffffff
#define FLT_MAX 3.402823466e+38F

#define PARA_PERIOD 45

#ifdef _DEBUG
#define PRINT   printf
#else
#define PRINT(...)
#endif

enum
{// ��������
	FORWARD,	// ����
	BACKWARD,	// ����
};

enum
{// �ó����Ƿ���·�ڣ������·�ڽ�Ҫ���ı���ʻ��
	NONE, //����·��
	DD,	// ֱ�� //�����׹ٷ�Ϊʲô��D��ĸ��ʾֱ��
	LEFT,	// ��ת
	RIGHT,  //��ת
};

enum
{// ��������״̬ //��ο���̳�й���������ȵĽ���
	SLEEPING,	// �ȴ������������У�
	WAITTING,	// �ȴ���ʻ 
	FINESHED,   // ��ֹ���� 
};


#endif
