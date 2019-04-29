#ifndef GRAPH_H
#define GRAPH_H
#include "data_pre_processing.h"
#include <limits>
//extern int s_para;
//记录起点到每个顶点的最短路径的信息
struct Dis {
    string path;
    int value;
    bool visit;
    Dis() {
        visit = false;
        value = 0;
        path = "";
    }
};

class Graph_DG
{
public:
    int vexnum;//图的顶点个数
    int edge;//图的边数
    int ** arc;//邻接矩阵
    Dis *dis; //记录各个顶点最短路径的信息
    Graph_DG(int vexnum);
    ~Graph_DG();
    void CreateGraph(vector<Road> &m_road);
    void Dijkstra(int begin);//算法实现，求解最优路径
    void print_path(int begin);//打印最优路径
    void lJMat_print();//打印邻接矩阵
    vector<vector<int>> Get_Vex_path(vector<int> &value,Graph_DG &Map,vector<Road> &m_road, int Gstart_id);//获取最优路径的顶点集合
};

#endif // GRAPH_H
