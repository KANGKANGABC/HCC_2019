#include "dijkstra.h"
#include "iostream"
#include "DataCenter.h"
#include "fstream"
using namespace std;

//���캯������
Graph_DG::Graph_DG(int vexnum, int edge):
vexnum(vexnum), edge(edge){
	//��ʼ��·�����ڽӾ���
	arc = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++) {
		arc[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//��ʼ��Ϊ�����
			arc[i][k] = INT_MAX;
		}
	}
	//��ʼ����·���ٵ��ڽӾ���
	arcRoadv = new int*[this->vexnum];
	for (int i = 0; i < this->vexnum; i++) {
		arcRoadv[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//��ʼ��Ϊ�����
			arcRoadv[i][k] = INT_MAX;
		}
	}
	//��ʼ��ʱ����ڽӾ���
	arcTime = new float*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++) {
		arcTime[i] = new float[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//��ʼ��Ϊ�����
			arcTime[i][k] = FLT_MAX;
		}
	}
}

//��������
Graph_DG::~Graph_DG() {
	//delete[] dis;
	for (int i = 0; i < this->vexnum; i++) {
		delete this->arc[i];
	}
	delete arc;

	for (int i = 0; i < this->vexnum; i++) {
		delete this->arcRoadv[i];
	}
	delete arcRoadv;

	for (int i = 0; i < this->vexnum; i++) {
		delete this->arcTime[i];
	}
	delete arcTime;
}


//����·�����ڽӾ���ͼ
void Graph_DG::createArcGraph(vector<std::vector<int> > graphRoad) {
	for (int i = 0; i < graphRoad.size(); i++)
	{
		for (int j = 0; j < graphRoad[0].size(); j++)
		{
			if (graphRoad[i][j] == 0)
				arc[i][j] = INT_MAX;
			else
			{
				arc[i][j] = graphRoad[i][j];
			}
		}
	}
}

//������·���ٵ��ڽӾ���ͼ
void Graph_DG::createArcRoadvGraph(vector<std::vector<int> > graphMaxSpeed) {
	for (int i = 0; i < graphMaxSpeed.size(); i++)
	{
		for (int j = 0; j < graphMaxSpeed[0].size(); j++)
		{
			if (graphMaxSpeed[i][j] == 0)
				arcRoadv[i][j] = 0;
			else
			{
				arcRoadv[i][j] = graphMaxSpeed[i][j];
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

//��ӡ��·�����ڽӾ���
void Graph_DG::printRoadv() {
	cout << "��·�����ڽӾ���Ϊ��" << endl;
	int count_row = 0;	//��ӡ�б�ǩ
	int count_col = 0;	//��ӡ�б�ǩ
	//��ʼ��ӡ
	while (count_row != this->vexnum) {
		count_col = 0;
		while (count_col != this->vexnum) {
			if (arcRoadv[count_row][count_col] == 0)
				cout << "0" << " ";
			else
				cout << arcRoadv[count_row][count_col] << " ";
			++count_col;
		}
		cout << endl;
		++count_row;
	}
}


//��ӡʱ���ڽӾ���
void Graph_DG::printTimeArc() {
	cout << "ʱ���ڽӾ���Ϊ��" << endl;
	int count_row = 0;	//��ӡ�б�ǩ
	int count_col = 0;	//��ӡ�б�ǩ
	//��ʼ��ӡ
	while (count_row != this->vexnum) {
		count_col = 0;
		while (count_col != this->vexnum) {
			if (arcTime[count_row][count_col] == FLT_MAX)
				cout << "��" << " ";
			else
				cout << arcTime[count_row][count_col] << " ";
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
				vector<int> tmp;
				tmp = dis[temp].path;
				tmp.push_back(i + 1);
				dis[i].path = tmp;
			}
		}
	}
}

//������ֵ������start��end��·��ֵ
vector<int> Graph_DG::Dijkstra(int begin, int end) {
	//���ȳ�ʼ��dis����
	dis = new Dis[this->vexnum];
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
				vector<int> tmp;
				tmp = dis[temp].path;
				tmp.push_back(i + 1);
				dis[i].path = tmp;
			}
		}
	}
	vector<int> path_tmp = dis[end - 1].path;
	delete[] dis;
	return path_tmp;
}

//����dijkstra�㷨�����ش�start��end��ʱ����̵�·��
vector<int> Graph_DG::Dijkstra(int begin, int end, int speed) {
	//���ȳ�ʼ��dis����
	disfloat = new DisFloat[this->vexnum];

	//ͳ�Ʒ�����flag[]���鸺��ͳ��cross�Ĺ滮ʹ����������Եõ���·��ʹ���ʣ�ӵ���������flagnum�����¼ͳ�ƵĴ�����ÿ100�μ�¼һ��
	static int flag[100] = { 0 };
	static int flagnum = 0;

	//����ʱ����ڽӾ���
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			if (arc[i][j] == INT_MAX || arcRoadv[i][j] == 0 || speed == 0)
			{
				arcTime[i][j] = FLT_MAX;
			}

			else
			{
				arcTime[i][j] = arcRoadv[i][j] > speed ? ((float)arc[i][j] / speed) : ((float)arc[i][j] / arcRoadv[i][j]);	//ȡ��·���ٺͳ��ٽ�С��һ��������ʱ��
			}
		}
	}

	int i;
	for (i = 0; i < this->vexnum; i++) {
		//���õ�ǰ��·��
		vector<int> tmp;
		tmp = disfloat[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		disfloat[i].path = tmp;
		disfloat[i].value = arc[begin - 1][i];	//���ڽ�����������һ�е�ֵ����dis����
	}
	//������㵽����Լ���·��Ϊ0
	disfloat[begin - 1].value = 0;
	disfloat[begin - 1].visit = true;

	int count = 1;
	//���㵽��������������·��
	while (count != this->vexnum) {
		//temp���ڱ��浱ǰdis��������С���Ǹ��±�
		//min��¼��ǰ����Сֵ
		int temp = 0;
		int min = INT_MAX;
		//�����forѭ���ҵ��������㷨�����ȶ��е����ã���Ŀǰ��̵ĵ㣩��ѡ��׼�������ɳڵĵ㣬����һ����forѭ�������ɳڲ���
		for (i = 0; i < this->vexnum; i++) {
			if (!disfloat[i].visit && disfloat[i].value < min) {
				min = disfloat[i].value;
				temp = i;
			}
		}

		disfloat[temp].visit = true;		//����һ���ҵ���׼�������ɳڲ����ĵ�������ҵ������·�����ϣ�ʵ���Ͼ����´β��������ȶ��У�ÿ����������һ����ӣ�
		++count;
		//�������forѭ����ô��⣨���µ���tempָ��ĵ��ֵ�������ǽ����еĵ㶼������һ���ɳڸ��µĲ��������������������·��򲻸��£����㷨��д����ֵ�������ڵĵ���в�������Ϊ�����ڵ�û�����壨���������������������������������ֱ�����е�������������ֻ���ڽӵĵ��ǻ���ã���
		for (i = 0; i < this->vexnum; i++) {
			if (!disfloat[i].visit && arc[temp][i] != INT_MAX && (disfloat[temp].value + arc[temp][i]) < disfloat[i].value) {
				disfloat[i].value = disfloat[temp].value + arc[temp][i];
				vector<int> tmp;
				tmp = disfloat[temp].path;
				tmp.push_back(i + 1);
				disfloat[i].path = tmp;
			}
		}
	}
	vector<int> path_tmp = disfloat[end - 1].path;

/********************************ͳ�Ƶ�·����ͨ���ʲ���**********************************/
	for (int i = 0; i < path_tmp.size(); i++) {
		//cout << path_tmp.size() <<" " << path_tmp.at(i) <<  endl;
		flag[path_tmp.at(i)]++;//��¼64��cross��ʹ�����
	}
	flagnum++;
	/*д��ǰ100������ͳ�����*/
	if (flagnum == 100)
	{
		ofstream oFile;
		oFile.open("test.csv", ios::out | ios::trunc);
		for (int i=0; i < 100; i++)
		{
			oFile << flag[i] << endl;
		}
		
		for (int i = 0; i < 100; i++)
		{
			flag[i] = 0;
		}
		oFile.close();
	}

	/*д��100~200������ͳ�����*/
	if (flagnum == 200)
	{
		ofstream oFile;
		oFile.open("test2.csv", ios::out | ios::trunc);
		for (int i = 0; i < 100; i++)
		{
			oFile << flag[i] << endl;
		}

		oFile.close();
	}
/********************************ͳ�Ƶ�·����ͨ���ʲ���**********************************/

	delete[] disfloat;//�ͷŶ�̬�����disfloat����
	return path_tmp;
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
			cout << "value= " << dis[i].value << endl;
		}
		else {
			for (int j = 0; j < dis[i].path.size(); j++)
				cout << dis[i].path.at(j) << " ";
			cout << "������������·����" << endl;
		}
	}
}


vector<int> Graph_DG::print_path(int begin, int end) {
	end = end - 1;
	cout << "��" << begin << "->��" << end << "��·��Ϊ";
	for (int i = 0; i < dis[end].path.size(); i++)
		cout << dis[end].path.at(i) << " ";
	cout << "���� " << dis[end].value << endl;

	return dis[end].path;
}