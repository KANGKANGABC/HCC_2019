#include "dijkstra.h"
#include "iostream"
#include "DataCenter.h"
using namespace std;

//构造函数定义
Graph_DG::Graph_DG(int vexnum, int edge):
vexnum(vexnum), edge(edge){
	//初始化路长的邻接矩阵
	arc = new int*[this->vexnum];		//指向指针的数组，实际就是一个二维的数组（矩阵）
	for (int i = 0; i < this->vexnum; i++) {
		arc[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//初始化为无穷达
			arc[i][k] = INT_MAX;
		}
	}
	//初始化道路限速的邻接矩阵
	arcRoadv = new int*[this->vexnum];
	for (int i = 0; i < this->vexnum; i++) {
		arcRoadv[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//初始化为无穷达
			arcRoadv[i][k] = INT_MAX;
		}
	}
	//初始化时间的邻接矩阵
	arcTime = new float*[this->vexnum];		//指向指针的数组，实际就是一个二维的数组（矩阵）
	for (int i = 0; i < this->vexnum; i++) {
		arcTime[i] = new float[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//初始化为无穷达
			arcTime[i][k] = FLT_MAX;
		}
	}

	//初始化jamDegree的邻接矩阵
	jamDegree = new int*[this->vexnum];		//指向指针的数组，实际就是一个二维的数组（矩阵）
	for (int i = 0; i < this->vexnum; i++) {
		jamDegree[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//初始化为0
			jamDegree[i][k] = 0;
		}
	}
	//初始化jamDegreeTmp的邻接矩阵
	jamDegreeTmp = new int*[this->vexnum];		//指向指针的数组，实际就是一个二维的数组（矩阵）
	for (int i = 0; i < this->vexnum; i++) {
		jamDegreeTmp[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//初始化为0
			jamDegreeTmp[i][k] = 0;
		}
	}
}

//析构函数
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


//创建路长的邻接矩阵图
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

//创建道路限速的邻接矩阵图
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

//打印邻接矩阵
void Graph_DG::print() {
	cout << "邻接矩阵为：" << endl;
	int count_row = 0;	//打印行标签
	int count_col = 0;	//打印列标签
	//开始打印
	while (count_row != this->vexnum) {
		count_col = 0;
		while (count_col != this->vexnum) {
			if (arc[count_row][count_col] == INT_MAX)
				cout << "∞" << " ";
			else
				cout << arc[count_row][count_col] << " ";
			++count_col;
		}
		cout << endl;
		++count_row;
	}
}

//打印道路限速邻接矩阵
void Graph_DG::printRoadv() {
	cout << "道路限速邻接矩阵为：" << endl;
	int count_row = 0;	//打印行标签
	int count_col = 0;	//打印列标签
	//开始打印
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


//打印时间邻接矩阵
void Graph_DG::printTimeArc() {
	cout << "时间邻接矩阵为：" << endl;
	int count_row = 0;	//打印行标签
	int count_col = 0;	//打印列标签
	//开始打印
	while (count_row != this->vexnum) {
		count_col = 0;
		while (count_col != this->vexnum) {
			if (arcTime[count_row][count_col] == FLT_MAX)
				cout << "∞" << " ";
			else
				cout << arcTime[count_row][count_col] << " ";
			++count_col;
		}
		cout << endl;
		++count_row;
	}
}

void Graph_DG::Dijkstra(int begin) {
	//首先初始化dis数组
	int i;
	for (i = 0; i < this->vexnum; i++) {
		//设置当前的路径
		vector<int> tmp;
		tmp = dis[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		dis[i].path = tmp;
		dis[i].value = arc[begin - 1][i];	//将邻接数组起点的那一行的值赋给dis数组
	}
	//设置起点到起点自己的路径为0
	dis[begin - 1].value = 0;
	dis[begin - 1].visit = true;

	int count = 1;
	//计算到其他各顶点的最短路径
	while (count != this->vexnum) {
		//temp用于保存当前dis数组中最小的那个下标
		//min记录当前的最小值
		int temp = 0;
		int min = INT_MAX;
		//这里的for循环我的理解就是算法中优先队列的作用（找目前最短的点），选择准备进行松弛的点，给下一步的for循环进行松弛操作
		for (i = 0; i < this->vexnum; i++) {
			if (!dis[i].visit && dis[i].value < min) {
				min = dis[i].value;
				temp = i;
			}
		}

		dis[temp].visit = true;		//把上一步找到的准备进行松弛操作的点加入已找到的最短路径集合（实际上就是下次不再入优先队列，每个点至进行一次入队）
		++count;
		//下面这个for循环这么理解（更新的是temp指向的点的值），它是将所有的点都进行了一次松弛更新的操作，如果满足条件则更新否则不更新，在算法中写的是值操作相邻的点进行操作，因为不相邻的没有意义（这里就用无穷达来表达了这种情况！因此它直接所有点遍历，如果可以只存邻接的点那会更好！）
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

//带返回值，返回start到end的路径值
vector<int> Graph_DG::Dijkstra(int begin, int end) {
	//首先初始化dis数组
	dis = new Dis[this->vexnum];
	int i;
	for (i = 0; i < this->vexnum; i++) {
		//设置当前的路径
		vector<int> tmp;
		tmp = dis[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		dis[i].path = tmp;
		dis[i].value = arc[begin - 1][i];	//将邻接数组起点的那一行的值赋给dis数组
	}
	//设置起点到起点自己的路径为0
	dis[begin - 1].value = 0;
	dis[begin - 1].visit = true;

	int count = 1;
	//计算到其他各顶点的最短路径
	while (count != this->vexnum) {
		//temp用于保存当前dis数组中最小的那个下标
		//min记录当前的最小值
		int temp = 0;
		int min = INT_MAX;
		//这里的for循环我的理解就是算法中优先队列的作用（找目前最短的点），选择准备进行松弛的点，给下一步的for循环进行松弛操作
		for (i = 0; i < this->vexnum; i++) {
			if (!dis[i].visit && dis[i].value < min) {
				min = dis[i].value;
				temp = i;
			}
		}

		dis[temp].visit = true;		//把上一步找到的准备进行松弛操作的点加入已找到的最短路径集合（实际上就是下次不再入优先队列，每个点至进行一次入队）
		++count;
		//下面这个for循环这么理解（更新的是temp指向的点的值），它是将所有的点都进行了一次松弛更新的操作，如果满足条件则更新否则不更新，在算法中写的是值操作相邻的点进行操作，因为不相邻的没有意义（这里就用无穷达来表达了这种情况！因此它直接所有点遍历，如果可以只存邻接的点那会更好！）
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

//重载dijkstra算法，返回从start到end的时间最短的路径
vector<int> Graph_DG::Dijkstra(int begin, int end, int speed) {
	//首先初始化dis数组
	disfloat = new DisFloat[this->vexnum];

	static float w =0.1;

	//计算时间的邻接矩阵
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
				arcTime[i][j] = arcRoadv[i][j] > speed ? (((float)arc[i][j] / speed) + w * jamDegree[i][j]) : (((float)arc[i][j] / arcRoadv[i][j]) + w * jamDegree[i][j]);	//取道路限速和车速较小的一个用来求时间
			}
		}
	}

	int i;
	for (i = 0; i < this->vexnum; i++) {
		//设置当前的路径
		vector<int> tmp;
		tmp = disfloat[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		disfloat[i].path = tmp;
		disfloat[i].value = arcTime[begin - 1][i];	//将邻接数组起点的那一行的值赋给dis数组
	}
	//设置起点到起点自己的路径为0
	disfloat[begin - 1].value = 0;
	disfloat[begin - 1].visit = true;

	int count = 1;
	//计算到其他各顶点的最短路径
	while (count != this->vexnum) {
		//temp用于保存当前dis数组中最小的那个下标
		//min记录当前的最小值
		int temp = 0;
		float min = FLT_MAX;
		//这里的for循环我的理解就是算法中优先队列的作用（找目前最短的点），选择准备进行松弛的点，给下一步的for循环进行松弛操作
		for (i = 0; i < this->vexnum; i++) {
			if (!disfloat[i].visit && disfloat[i].value < min) {
				min = disfloat[i].value;
				temp = i;
			}
		}

		disfloat[temp].visit = true;		//把上一步找到的准备进行松弛操作的点加入已找到的最短路径集合（实际上就是下次不再入优先队列，每个点至进行一次入队）
		++count;
		//下面这个for循环这么理解（更新的是temp指向的点的值），它是将所有的点都进行了一次松弛更新的操作，如果满足条件则更新否则不更新，在算法中写的是值操作相邻的点进行操作，因为不相邻的没有意义（这里就用无穷达来表达了这种情况！因此它直接所有点遍历，如果可以只存邻接的点那会更好！）
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

	delete[] disfloat;//释放动态申请的disfloat数组
	return path_tmp;
}

vector<int> Graph_DG::Dijkstra(int begin, int end, int speed, vector<std::vector<float>> jamDegree, float w)
{
	//首先初始化dis数组
	disfloat = new DisFloat[this->vexnum];

	//计算时间的邻接矩阵
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
				arcTime[i][j] = arcRoadv[i][j] > speed ? (((float)arc[i][j] / speed) + w * jamDegree[i][j]) : (((float)arc[i][j] / arcRoadv[i][j]) + w * jamDegree[i][j]);	//取道路限速和车速较小的一个用来求时间
			}
		}
	}

	int i;
	for (i = 0; i < this->vexnum; i++) {
		//设置当前的路径
		vector<int> tmp;
		tmp = disfloat[i].path;
		tmp.push_back(begin);
		tmp.push_back(i + 1);
		disfloat[i].path = tmp;
		disfloat[i].value = arcTime[begin - 1][i];	//将邻接数组起点的那一行的值赋给dis数组
	}
	//设置起点到起点自己的路径为0
	disfloat[begin - 1].value = 0;
	disfloat[begin - 1].visit = true;

	int count = 1;
	//计算到其他各顶点的最短路径
	while (count != this->vexnum) {
		//temp用于保存当前dis数组中最小的那个下标
		//min记录当前的最小值
		int temp = 0;
		float min = FLT_MAX;
		//这里的for循环我的理解就是算法中优先队列的作用（找目前最短的点），选择准备进行松弛的点，给下一步的for循环进行松弛操作
		for (i = 0; i < this->vexnum; i++) {
			if (!disfloat[i].visit && disfloat[i].value < min) {
				min = disfloat[i].value;
				temp = i;
			}
		}

		disfloat[temp].visit = true;		//把上一步找到的准备进行松弛操作的点加入已找到的最短路径集合（实际上就是下次不再入优先队列，每个点至进行一次入队）
		++count;
		//下面这个for循环这么理解（更新的是temp指向的点的值），它是将所有的点都进行了一次松弛更新的操作，如果满足条件则更新否则不更新，在算法中写的是值操作相邻的点进行操作，因为不相邻的没有意义（这里就用无穷达来表达了这种情况！因此它直接所有点遍历，如果可以只存邻接的点那会更好！）
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

	delete[] disfloat;//释放动态申请的disfloat数组
	return path_tmp;
}

void Graph_DG::print_path(int begin) {
	string str;
	str = "v" + to_string(begin);
	cout << "以" << str << "为起点的图的最短路径为：" << endl;
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
			cout << "两点间是无最短路径的" << endl;
		}
	}
}


vector<int> Graph_DG::print_path(int begin, int end) {
	end = end - 1;
	cout << "点" << begin << "->点" << end << "的路径为";
	for (int i = 0; i < dis[end].path.size(); i++)
		cout << dis[end].path.at(i) << " ";
	cout << "长度 " << dis[end].value << endl;

	return dis[end].path;
}

void Graph_DG::upDateJam()
{
	//将临时jamDgreeTmp中的统计值更新到jamDegree中，用来给time邻接矩阵做调整
	for (int i = 0; i < this->vexnum; i++) 
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegree[i][j] = jamDegreeTmp[i][j];
		}
	}
	//将jamDegreeTmp的矩阵重置为0，准备下一次100辆车的统计信息
	for (int i = 0; i < this->vexnum; i++)
	{
		for (int j = 0; j < this->vexnum; j++)
		{
			jamDegreeTmp[i][j] = 0;
		}
	}
}
