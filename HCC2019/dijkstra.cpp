#include "dijkstra.h"
#include "iostream"
#include "DataCenter.h"
using namespace std;

//���캯������
Graph_DG::Graph_DG(int vexnum, int edge) {
	//��ʼ���������ͱ���
	this->vexnum = vexnum;
	this->edge = edge;
	//Ϊ�ڽӾ��󿪱ٿռ�͸���ֵ
	arc = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	dis = new Dis[this->vexnum];		//Dis�͵�һά����
	for (int i = 0; i < this->vexnum; i++) {
		arc[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//��ʼ��Ϊ�����
			arc[i][k] = INT_MAX;
		}
	}

}

//��������
Graph_DG::~Graph_DG() {
	delete[] dis;
	for (int i = 0; i < this->vexnum; i++) {
		delete this->arc[i];
	}
	delete arc;
}

// �ж�����ÿ������ĵıߵ���Ϣ�Ƿ�Ϸ�
//�����1��ʼ���
bool Graph_DG::check_edge_value(int start, int end, int weight) {
	if (start<1 || end<1 || start>vexnum || end>vexnum || weight < 0) {
		return false;
	}
	return true;
}

//����ͼ�Σ��������ڽӾ���,��Ҫ�ֶ�����
void Graph_DG::createGraph(vector<std::vector<int> > graphRoad) {
	//cout << "������ÿ���ߵ������յ��Լ�Ȩ��" << endl;
	//int start;
	//int end;
	//int weight;
	//int count = 0;
	//while (count != this->edge) {
	//	cin >> start >> end >> weight;
	//	while (!this->check_edge_value(start, end, weight)) {
	//		cout << "����ߵ���Ϣ���Ϸ�������������" << endl;
	//		cin >> start >> end >> weight;
	//	}
	//	//���ڽӾ����Ӧ�ĵ㸳ֵ
	//	arc[start - 1][end - 1] = weight;
	//	++count;
	//}

	for (int i = 0; i < graphRoad.size(); i++)
	{
		for (int j = 0; j < graphRoad[0].size(); j++)
		{
			if(graphRoad[i][j] == 0)
				arc[i][j] = INT_MAX;
			else
			{
				arc[i][j] = graphRoad[i][j];
			}
		}
	}
}

//��ӡ�ڽӾ���
void Graph_DG::print() {
	cout << "�ڽӾ���Ϊ��" << endl;
	int count_row = 0;	//��ӡ�б�ǩ
	int count_col = 0;	//��ӡ�б�ǩ
	//��ʼ��ӡ
	while (count_row != this->vexnum) {
		count_col = 0;
		while (count_col != this->vexnum) {
			if (arc[count_row][count_col] == INT_MAX)
				cout << "��" << " ";
			else
				cout << arc[count_row][count_col] << " ";
			++count_col;
		}
		cout << endl;
		++count_row;
	}
}

void Graph_DG::Dijkstra(int begin) {
	//���ȳ�ʼ��dis����
	int i;
	for (i = 0; i < this->vexnum; i++) {
		//���õ�ǰ��·��
		vector<int> tmp;
		tmp = dis[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		dis[i].path = tmp;
		dis[i].value = arc[begin - 1][i];	//���ڽ�����������һ�е�ֵ����dis����
	}
	//������㵽����Լ���·��Ϊ0
	dis[begin - 1].value = 0;
	dis[begin - 1].visit = true;

	int count = 1;
	//���㵽��������������·��
	while (count != this->vexnum) {
		//temp���ڱ��浱ǰdis��������С���Ǹ��±�
		//min��¼��ǰ����Сֵ
		int temp = 0;
		int min = INT_MAX;
		//�����forѭ���ҵ��������㷨�����ȶ��е����ã���Ŀǰ��̵ĵ㣩��ѡ��׼�������ɳڵĵ㣬����һ����forѭ�������ɳڲ���
		for (i = 0; i < this->vexnum; i++) {
			if (!dis[i].visit && dis[i].value < min) {
				min = dis[i].value;
				temp = i;
			}
		}
		
		dis[temp].visit = true;		//����һ���ҵ���׼�������ɳڲ����ĵ�������ҵ������·�����ϣ�ʵ���Ͼ����´β��������ȶ��У�ÿ����������һ����ӣ�
		++count;
		//�������forѭ����ô��⣨���µ���tempָ��ĵ��ֵ�������ǽ����еĵ㶼������һ���ɳڸ��µĲ��������������������·��򲻸��£����㷨��д����ֵ�������ڵĵ���в�������Ϊ�����ڵ�û�����壨���������������������������������ֱ�����е�������������ֻ���ڽӵĵ��ǻ���ã���
		for (i = 0; i < this->vexnum; i++) {
			if (!dis[i].visit && arc[temp][i] != INT_MAX && (dis[temp].value + arc[temp][i]) < dis[i].value) {
				dis[i].value = dis[temp].value + arc[temp][i];
				//dis[i].path = dis[temp].path + "-->" + to_string(i + 1);
				vector<int> tmp;
				tmp = dis[temp].path;
				tmp.push_back(i + 1);
				dis[i].path = tmp;
			}
		}
	}
}

void Graph_DG::print_path(int begin) {
	string str;
	str = "v" + to_string(begin);
	cout << "��" << str << "Ϊ����ͼ�����·��Ϊ��" << endl;
	for (int i = 0; i != this->vexnum; i++) {
		if (dis[i].value != INT_MAX)
		{
			for (int j = 0; j < dis[i].path.size(); j++)
				cout << dis[i].path.at(j) << " ";
			cout << "value= "<< dis[i].value << endl;
		}
		else {
			for (int j = 0; j < dis[i].path.size(); j++)
				cout << dis[i].path.at(j) << " ";
			cout << "������������·����" << endl;
		}
	}
}