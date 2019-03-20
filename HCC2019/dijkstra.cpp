#include "dijkstra.h"
#include "iostream"
#include "DataCenter.h"
#include<algorithm>
using namespace std;

//���캯������
Graph_DG::Graph_DG(int vexnum, int edge) :
	vexnum(vexnum), edge(edge) {
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

	//��ʼ��jamDegree���ڽӾ���
	jamDegree = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++) {
		jamDegree[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//��ʼ��Ϊ0
			jamDegree[i][k] = 0;
		}
	}
	//��ʼ��jamDegreeTmp���ڽӾ���
	jamDegreeTmp = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++) {
		jamDegreeTmp[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//��ʼ��Ϊ0
			jamDegreeTmp[i][k] = 0;
		}
	}

	//��ʼ��jamDegreeLongBefore���ڽӾ���
	jamDegreeLongBefore = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++)
	{
		jamDegreeLongBefore[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++)
		{
			//��ʼ��Ϊ0
			jamDegreeLongBefore[i][k] = 0;
		}
	}

	//��ʼ��jamDegreeBefore���ڽӾ���
	jamDegreeBefore = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++)
	{
		jamDegreeBefore[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++)
		{
			//��ʼ��Ϊ0
			jamDegreeBefore[i][k] = 0;
		}
	}

	//��ʼ��jamDegreeNowInt���ڽӾ���
	jamDegreeNowInt = new int*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++)
	{
		jamDegreeNowInt[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++)
		{
			//��ʼ��Ϊ0
			jamDegreeNowInt[i][k] = 0;
		}
	}

	//��ʼ��jamDegreeNowFloat���ڽӾ���
	jamDegreeNowFloat = new float*[this->vexnum];		//ָ��ָ������飬ʵ�ʾ���һ����ά�����飨����
	for (int i = 0; i < this->vexnum; i++)
	{
		jamDegreeNowFloat[i] = new float[this->vexnum];
		for (int k = 0; k < this->vexnum; k++)
		{
			//��ʼ��Ϊ0
			jamDegreeNowFloat[i][k] = 0;
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

//�ù�һ������ڽӾ���ʵʩdijkstra�㷨
vector<int> Graph_DG::DijkstraNor(int begin, int end, int speed)
{
	//���ȳ�ʼ��dis����
	disfloat = new DisFloat[this->vexnum];

	static float w = 50;

	//����ʱ���δ��һ���ڽӾ���
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			if (arc[i][j] == INT_MAX || arcRoadv[i][j] == 0 || speed == 0)
			{
				arcTime[i][j] = FLT_MAX;
				//cout << 0 << ' ';
			}
			else
			{
				arcTime[i][j] = (float)arc[i][j] / min(speed, arcRoadv[i][j]);	      //ȡ��·���ٺͳ��ٽ�С��һ��������ʱ��
				//cout << arcTime[i][j] << ' ';
			}
		}
		//cout << '\n' << endl;
	}

	//�õ���һ�����ʱ�����
	normalizedFloat(arcTime);

	//�õ�������ʱ��ͽ�ͨӵ�µĹ�һ������
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			if (arc[i][j] == INT_MAX || arcRoadv[i][j] == 0 || speed == 0)
			{
				arcTime[i][j] = FLT_MAX;
				//cout << 0 << ' ';

			}

			else
			{
				arcTime[i][j] = arcTime[i][j] + jamDegreeNowFloat[i][j];
				//cout << arcTime[i][j] << ' ';
			}
		}
		//cout << '\n' << endl;
	}

	int i;
	for (i = 0; i < this->vexnum; i++) {
		//���õ�ǰ��·��
		vector<int> tmp;
		tmp = disfloat[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		disfloat[i].path = tmp;
		disfloat[i].value = arcTime[begin - 1][i];	//���ڽ�����������һ�е�ֵ����dis����
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
		float min = FLT_MAX;
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
			if (!disfloat[i].visit && arcTime[temp][i] != FLT_MAX && (disfloat[temp].value + arcTime[temp][i]) < disfloat[i].value) {
				disfloat[i].value = disfloat[temp].value + arcTime[temp][i];
				vector<int> tmp;
				tmp = disfloat[temp].path;
				tmp.push_back(i + 1);
				disfloat[i].path = tmp;
			}
		}
	}
	vector<int> path_tmp = disfloat[end - 1].path;

	delete[] disfloat;//�ͷŶ�̬�����disfloat����
	return path_tmp;
}

//����dijkstra�㷨�����ش�start��end��ʱ����̵�·��
vector<int> Graph_DG::Dijkstra(int begin, int end, int speed) {
	//���ȳ�ʼ��dis����
	disfloat = new DisFloat[this->vexnum];

	static float w = 50;

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

				arcTime[i][j] = arcRoadv[i][j] > speed ? (((float)arc[i][j] / speed) + w * jamDegree[i][j]) : (((float)arc[i][j] / arcRoadv[i][j]) + w * jamDegree[i][j]);	//ȡ��·���ٺͳ��ٽ�С��һ��������ʱ��

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
		disfloat[i].value = arcTime[begin - 1][i];	//���ڽ�����������һ�е�ֵ����dis����
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
		float min = FLT_MAX;
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
			if (!disfloat[i].visit && arcTime[temp][i] != FLT_MAX && (disfloat[temp].value + arcTime[temp][i]) < disfloat[i].value) {
				disfloat[i].value = disfloat[temp].value + arcTime[temp][i];
				vector<int> tmp;
				tmp = disfloat[temp].path;
				tmp.push_back(i + 1);
				disfloat[i].path = tmp;
			}
		}
	}
	vector<int> path_tmp = disfloat[end - 1].path;

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

void Graph_DG::upDateJam()
{
	//����ʱjamDgreeTmp�е�ͳ��ֵ���µ�jamDegree�У�������time�ڽӾ���������
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegree[i][j] = jamDegreeTmp[i][j];
		}
	}
	//��jamDegreeTmp�ľ�������Ϊ0��׼����һ��100������ͳ����Ϣ
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegreeTmp[i][j] = 0;
		}
	}
}

void Graph_DG::upDateJamStatic() 
{
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegreeLongBefore[i][j] = jamDegreeBefore[i][j];
		}
	}
}

void Graph_DG :: cleanUpJamDegreeBefore()
{
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegreeBefore[i][j] = 0;
		}
	}

}

void  Graph_DG::upDateJamDynamic()
{
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegreeNowInt[i][j] = jamDegreeLongBefore[i][j] + jamDegreeBefore[i][j];
		}
	}

//��һ��
	normalizedInt(jamDegreeNowInt, jamDegreeNowFloat);
}

void  Graph_DG::normalizedInt( int**temp ,  float **tempNormalized)
{
	vector<int> tempValue;
	int tempMax = 0;
	int tempMin = 0;
	int tempDifference=0;

	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			tempValue.push_back(temp[i][j]);
		}
	}

	tempMax = *max_element(tempValue.begin(),tempValue.end());
	tempDifference = tempMax - tempMin;

	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			tempNormalized[i][j] = tempDifference == 0 ? 0 : float((temp[i][j] - tempMin) )/ tempDifference;
			//cout << tempNormalized[i][j] << ' ' ;
		}
		//cout << '\n' << endl;
	}

}

void  Graph_DG::normalizedFloat( float**temp )   //��һ������ԭ����
{
	vector<float> tempValue;
	float tempMax = 0;
	float tempMin = 0;
	float tempDifference = 0;

	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			if (temp[i][j] != FLT_MAX)
			{
				tempValue.push_back(temp[i][j]);
			}	
		}
	}

	tempMax = *max_element(tempValue.begin(), tempValue.end());
	tempMin = *min_element(tempValue.begin(), tempValue.end());
	tempDifference = tempMax - tempMin;

	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			if (temp[i][j] != FLT_MAX)
			{
				temp[i][j] = (temp[i][j] - tempMin) / tempDifference;
				//cout << temp[i][j] << ' ';
			}
			//else 
				//cout << 0 << ' ';
		}
		//cout << '\n' << endl;
	}

}
