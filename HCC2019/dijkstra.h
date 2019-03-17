#pragma once

#include<iostream>
#include<string>
#include<vector>
#include "define.h"
using namespace std;


struct Dis {
	vector<int> path;//路径的经过
	int value;
	bool visit;
	Dis() {
		visit = false;	//判断是否已经被访问
		value = 0;		//路径的长度
	}
};

struct DisFloat {
	vector<int> path;//路径的经过
	float value;
	bool visit;
	DisFloat() {
		visit = false;	//判断是否已经被访问
		value = 0;		//路径的长度
	}
};

class Graph_DG {
private:
	int vexnum;		//图的顶点个数
	int edge;		//图的边数
	int **arc;		//邻接矩阵
	vector<vector<float> > *speedarc;	//存放各车速在道路上对应的速度邻接矩阵
	Dis *dis;		//记录各个顶点的最短路径信息
	
public:
	//构造函数（参数表：邻接矩阵，顶点数（cross数量），边数（road数量））
	Graph_DG(int vexnum, int edge);
	//析构函数
	~Graph_DG();
	//创建各种车速的邻接矩阵,输入参数（定点个数，车速种类数，道路的最大限速邻接矩阵）
	void creatAllSpeedGraph(int vexnum, int car_speed_num, vector<int> speedType, vector<vector<int> > graphMaxSpeed);
	//创建图
	void createGraph(vector<std::vector<int> > graphRoad);
	//打印邻接矩阵
	void print();
	//利用dijkstra算法求最短路径，输入起点
	void Dijkstra(int begin);
	//dijkstra算法，返回从start到end的路径
	vector<int> Dijkstra(int begin, int end);
	//打印begin到所有顶点的最短路径
	void print_path(int begin);
	//打印begin到end之间的最短距离
	vector<int> print_path(int begin, int end);
};
