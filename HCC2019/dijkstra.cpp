#include "dijkstra.h"
#include "iostream"
using namespace std;

//构造函数定义
Graph_DG::Graph_DG(int vexnum, int edge) {
	//初始化顶点数和边数
	this->vexnum = vexnum;
	this->edge = edge;
	//为邻接矩阵开辟空间和赋初值
	arc = new int*[this->vexnum];		//指向指针的数组，实际就是一个二维的数组（矩阵）
	dis = new Dis[this->vexnum];		//Dis型的一维数组
	for (int i = 0; i < this->vexnum; i++) {
		arc[i] = new int[this->vexnum];
		for (int k = 0; k < this->vexnum; k++) {
			//初始化为无穷达
			arc[i][k] = INT_MAX;
		}
	}
}

//析构函数
Graph_DG::~Graph_DG() {
	delete[] dis;
	for (int i = 0; i < this->vexnum; i++) {
		delete this->arc[i];
	}
	delete arc;
}

// 判断我们每次输入的的边的信息是否合法
//顶点从1开始编号
bool Graph_DG::check_edge_value(int start, int end, int weight) {
	if (start<1 || end<1 || start>vexnum || end>vexnum || weight < 0) {
		return false;
	}
	return true;
}

//创建图形，即建立邻接矩阵,需要手动输入
void Graph_DG::createGraph() {
	cout << "请输入每条边的起点和终点以及权重" << endl;
	int start;
	int end;
	int weight;
	int count = 0;
	while (count != this->edge) {
		cin >> start >> end >> weight;
		while (!this->check_edge_value(start, end, weight)) {
			cout << "输入边的信息不合法，请重新输入" << endl;
			cin >> start >> end >> weight;
		}
		//对邻接矩阵对应的点赋值
		arc[start - 1][end - 1] = weight;
		++count;
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

void Graph_DG::Dijkstra(int begin) {
	//首先初始化dis数组
	int i;
	for (i = 0; i < this->vexnum; i++) {
		//设置当前的路径
		vector<int> tmp;
		//dis[i].path = "v" + to_string(begin) + "-->v" + to_string(i + 1);
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
	cout << "以" << str << "为起点的图的最短路径为：" << endl;
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
			cout << "亮点间是无最短路径的" << endl;
		}
	}
}