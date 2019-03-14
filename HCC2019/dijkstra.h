#pragma once

#include<iostream>
#include<string>
#include<vector>
using namespace std;


struct Dis {
	vector<int> path;
	int value;
	bool visit;
	Dis() {
		visit = false;	//判断是否已经被访问
		value = 0;		//路径的长度
		//path = "";		//路径的经过
	}
};

class Graph_DG {
private:
	int vexnum;		//图的顶点个数
	int edge;		//图的边数
	int **arc;		//邻接矩阵
	Dis *dis;		//记录各个顶点的最短路径信息
	
public:
	//构造函数（参数表：邻接矩阵，顶点数（cross数量），边数（road数量））
	Graph_DG(int vexnum, int edge);
	//析构函数
	~Graph_DG();
	// 判断我们每次输入的的边的信息是否合法
	//顶点从1开始编号
	bool check_edge_value(int start, int end, int weight);
	//创建图
	void createGraph(vector<std::vector<int> > graphRoad);
	//打印邻接矩阵
	void print();
	//利用dijkstra算法求最短路径，输入起点
	void Dijkstra(int begin);
	//打印最短路径
	void print_path(int);
};