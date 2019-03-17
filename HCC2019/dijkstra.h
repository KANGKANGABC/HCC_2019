#pragma once

#include<iostream>
#include<string>
#include<vector>
#include "define.h"
using namespace std;


struct Dis {
	vector<int> path;//·���ľ���
	int value;
	bool visit;
	Dis() {
		visit = false;	//�ж��Ƿ��Ѿ�������
		value = 0;		//·���ĳ���
	}
};

class Graph_DG {
private:
	int vexnum;		//ͼ�Ķ������
	int edge;		//ͼ�ı���
	int **arc;		//�ڽӾ���
	Dis *dis;		//��¼������������·����Ϣ
	
public:
	//���캯�����������ڽӾ��󣬶�������cross��������������road��������
	Graph_DG(int vexnum, int edge);
	//��������
	~Graph_DG();
	// �ж�����ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
	//�����1��ʼ���
	bool check_edge_value(int start, int end, int weight);
	//����ͼ
	void createGraph(vector<std::vector<int> > graphRoad);
	//��ӡ�ڽӾ���
	void print();
	//����dijkstra�㷨�����·�����������
	void Dijkstra(int begin);
	//dijkstra�㷨�����ش�start��end��·��
	vector<int> Dijkstra(int begin, int end);
	//��ӡbegin�����ж�������·��
	void print_path(int begin);
	//��ӡbegin��end֮�����̾���
	vector<int> print_path(int begin, int end);
};
