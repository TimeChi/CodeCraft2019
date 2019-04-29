#include "graph.h"

//int s_para;

/*
*函数名称：Graph_DG默认构造函数
*函数功能：初始化邻接矩阵
*函数参数：图的顶点数vexnum、图的边数edge
*函数返回值：无
*Author:学学没完
*date：2019.3.11
*/
Graph_DG::Graph_DG(int vexnum)
{
    //初始化顶点数和边数
    this->vexnum = vexnum;
    //this->edge = edge;
    //为邻接矩阵开辟空间和赋初值
    arc = new int*[this->vexnum];
    dis = new Dis[this->vexnum];
    for(int i = 0; i < this->vexnum;i++)
    {
        arc[i] = new int[this->vexnum];
        for(int k = 0; k < this->vexnum; k++)
        {
            arc[i][k] = __INT_MAX__;//邻接矩阵初始化无穷大
        }
    }
}

Graph_DG::~Graph_DG()
{
    delete[] dis;
    for(int i =0; i < this->vexnum; i++)
    {
        delete this->arc[i];
    }
    delete arc;
}


void Graph_DG::CreateGraph(vector<Road> &m_road)
{
    int start;
    int end;
    for(uint count = 0; count < m_road.size(); count++)
    {
        start = m_road[count].start_ind;
        end = m_road[count].end_ind;
        if(m_road[count].isDuplex==1)
        {
            arc[start][end] = m_road[count].len_weight;
            arc[end][start] = m_road[count].len_weight;
        }else
        {
            arc[start][end] = m_road[count].len_weight*15;//*s_para;
        }
    }
}

/*
*函数名称：Dijkstra
*函数功能：迪杰斯特拉算法的实现，寻找出最优路径
*函数参数：起始点begin
*函数返回值：无
*Author:学学没完
*date：2019.3.11
*/
void Graph_DG::Dijkstra(int begin)
{
    //首先初始化我们的dis数组
    int i;
    for (i = 0; i < this->vexnum; i++)
    {
        //设置当前的路径
        dis[i].path = to_string(begin) + "-" + to_string(i);
        dis[i].value = arc[begin][i];
    }
    //设置起点到起点的路径为0
    dis[begin].value = 0;//dis[a].value为begin到a的距离
    dis[begin].visit =true;//标记未确定顶点中最小的为ture

    int count = 1;
    //计算剩余的顶点的最短路径（剩余this->vexnum-1个顶点）
    while(count != this->vexnum)
    {
        int temp = 0;//用于保存当前dis数组中最小的那个下标
        int min = __INT_MAX__;//记录当前的最小值
        for (i = 0; i < this->vexnum; i++)
        {
            if(!dis[i].visit && dis[i].value < min)
            {
                min = dis[i].value;
                temp = i;
            }
        }
        //把temp对应的顶点加入到已经找到的最短路径的集合中
        dis[temp].visit = true;
        ++count;
        for (i = 0; i < this->vexnum; i++)
        {
            //注意这里的条件arc[temp][i]!=INT_MAX必须加，不然会出现溢出，从而造成程序异常
            if(!dis[i].visit && arc[temp][i]!=__INT_MAX__ && (dis[temp].value + arc[temp][i]) < dis[i].value)
            {
                //如果新得到的边可以影响其他为访问的顶点，那就更新它的最短路径和长度
                dis[i].value = dis[temp].value + arc[temp][i];
                dis[i].path = dis[temp].path + "-" + to_string(i);
            }
        }

    }

}

/*
*函数名称：print_path
*函数功能：打印最优路径
*函数参数：起始点begin
*函数返回值：无
*Author:学学没完
*date：2019.3.11
*/
void Graph_DG::print_path(int begin) {
    string str;
    str = "v" + to_string(begin);
    cout << "以"<<str<<"为起点的图的最短路径为：" << endl;
    for (int i = 0; i != this->vexnum; i++)
    {
        if(dis[i].value!=__INT_MAX__)
        {
            cout << dis[i].path << "=" << dis[i].value << endl;
        }else
        {
            cout << dis[i].path << "是无最短路径的" << endl;
        }
    }
}

/*
*函数名称：Get_Vex_path
*函数功能：获取出发点到其他点之间的最优路径的顶点集合
*函数参数：地图Map，路信息road,出发地Gstart_id
*函数返回值：最优路径顶点集合path
*Author:学学没完
*date：2019.3.12
*/
vector<vector<int>> Graph_DG::Get_Vex_path(vector<int> &value, Graph_DG &Map, vector<Road> &m_road,int Gstart_id)
{
    vector<int> path;
    vector<vector<int>> s_path;

    Map.CreateGraph(m_road);
    Map.Dijkstra(Gstart_id);
    for (int j = 0; j< vexnum; j++)
    {

        vector<string> v;
        SplitString(dis[j].path, v, "-");
        for (uint i = 0; i< v.size(); i++)
        {
            path.push_back(atoi(v[i].c_str()));    
        }
        value.push_back(dis[j].value);
        s_path.push_back(path);
        path.clear();
    }
    return s_path;
}

/*
*函数名称：lJMat_print
*函数功能：打印邻接矩阵
*函数参数：无
*函数返回值：无
*Author:学学没完
*date：2019.3.11
*/
void Graph_DG::lJMat_print() {
    cout << "图的邻接矩阵为：" << endl;
    int count_row = 0; //打印行的标签
    int count_col = 0; //打印列的标签
    //开始打印
    while (count_row != this->vexnum) {
        count_col = 0;
        while (count_col != this->vexnum) {
            if (arc[count_row][count_col] == __INT_MAX__)
                cout << "∞" << " ";
            else
            cout << arc[count_row][count_col] << " ";
            ++count_col;
        }
        cout << endl;
        ++count_row;
    }
}


