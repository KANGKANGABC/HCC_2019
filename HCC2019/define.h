#ifndef DEFINE_H_
#define DEFINE_H_

#include <string>
#include <cstring>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <vector>

#define MAX_ROAD_NUM    100 //���ROAD����
#define MAX_CAR_NUM    150 //���CAR����
#define MAX_CROSS_NUM    100 //���CROSS����

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


#endif