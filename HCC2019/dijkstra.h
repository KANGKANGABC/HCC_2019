#pragma once

#include<iostream>
#include<string>
using namespace std;


struct Dis {
	std::vector<int> path ;
	float value;
	bool visit;
	Dis() {
		visit = false;	//�ж��Ƿ��Ѿ�������
		value = 0;		//·���ĳ���
							//·���ľ���
	}
};

class Graph_DG {
private:
	int vexnum;		//ͼ�Ķ������
	Dis *dis;		//��¼������������·����Ϣ
	
public:
	//���캯��
	Graph_DG(int vexnum, int edge);
	//��������
	~Graph_DG();
	// �ж�����ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
	//�����1��ʼ���
	bool check_edge_value(int start, int end, int weight);
	//����ͼ
	void createGraph();
	//��ӡ�ڽӾ���
	void print();
	//����dijkstra�㷨�����·�����������
	void Dijkstra(int begin);
	//��ӡ���·��
	void print_path(int);
};