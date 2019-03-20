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
	int **arc;		//路长的邻接矩阵
	int **arcRoadv;	//道路限速的邻接矩阵（直接将center.h中的graphMaxSpeed作为值即可）
	float **arcTime;	//时间的权重的邻接矩阵（在路径规划算法中做具体赋值）
	Dis *dis;		//记录各个顶点的路长的最短路径信息
	DisFloat *disfloat; //记录各个顶点的时间的最短路径信息

public:
	int **jamDegree;	//两个保存道路拥挤度的矩阵
	int **jamDegreeTmp;

	//用于根据前车运行情况更新邻接矩阵的getPathByTime_dynamic()的三个矩阵;
	int ** jamDegreeLongBefore;  //存储0-200辆车规划后的道路情况
	int ** jamDegreeBefore;          //存储200-i辆车规划后的道路情况
	int ** jamDegreeNowInt;         //存储前200+i辆车规划后的道路情况
	float ** jamDegreeNowFloat;   //存储归一化后的交通拥堵矩阵

	//构造函数（参数表：邻接矩阵，顶点数（cross数量），边数（road数量））
	Graph_DG(int vexnum, int edge);
	//析构函数
	~Graph_DG();
	//创建路长的邻接矩阵图
	void createArcGraph(vector<std::vector<int> > graphRoad);
	//创建道路限速的邻接矩阵图
	void createArcRoadvGraph(vector<std::vector<int> > graphMaxSpeed);
	//打印路长邻接矩阵
	void print();
	//打印道路限速邻接矩阵  
	void printRoadv();
	//打印时间邻接矩阵
	void printTimeArc();
	//利用dijkstra算法求最短路径，输入起点
	void Dijkstra(int begin);
	//dijkstra算法，返回从start到end的路径
	vector<int> Dijkstra(int begin, int end);
	//重载dijkstra算法，返回从start到end的时间最短的路径
	vector<int> Dijkstra(int begin, int end, int speed);

	//重载dijkstra算法，返回从start到end的时间最短的路径，jamDegree为道路拥堵情况矩阵，w为加权系数
	vector<int> Dijkstra(int begin, int end, int speed, vector<std::vector<float> > jamDegree, float w);
	//用归一化后的矩阵实施D算法
	vector<int> DijkstraNor(int begin, int end, int speed);
	//打印begin到所有顶点的最短路径
	void print_path(int begin);
	//打印begin到end之间的最短距离
	vector<int> print_path(int begin, int end);
	//跟新jamDegree的邻接矩阵
	void upDateJam();
	//更新jamDegreeLongBefore的邻接矩阵
	void upDateJamStatic();
	//将JamDegreeBefore矩阵清0
	void cleanUpJamDegreeBefore();
	//根据jamDegreeLongBefore和JamDegreeBefore的值求得jamDegreeNowInt的值，归一化得jamDegreeNowFloat
	void upDateJamDynamic();
	//归一化
	void normalizedInt( int ** temp , float **tempNormalized);
	void normalizedFloat(float **temp);
	//更新
	//清空
};
