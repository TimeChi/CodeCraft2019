#include"zzcode.h"
#include "graph.h"
#include<stdlib.h>
extern int tu;
/*
*函数名称：Get_dense_iterval
*函数功能：获取m_pcar车辆的稠密时间区域
*函数参数：m_pcar
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
vector<int> Get_dense_iterval(vector<presetCar> &m_pcar)
{
    int car_dense_th = 5;//每个时刻发车判断为稠密的阈值
    int time_point_th =30;//时间点间隔稠密的阈值
    int maxtemp = 0;
    for(auto iter=m_pcar.begin(); iter!=m_pcar.end(); iter++)
    {
        if(maxtemp < iter->go_time_PA) maxtemp = iter->go_time_PA;
    }
    vector<int> Time_go_num(maxtemp+1,0);
    for(auto iter=m_pcar.begin(); iter!=m_pcar.end(); iter++)
    {
        Time_go_num[iter->go_time_PA]++;
    }
    vector<int> fro_last_list;//存放所有稠密的时间点在Time_go_num中的索引
    for(int i=0; i < Time_go_num.size(); i++)
    {
        if(Time_go_num[i] > car_dense_th)
        {
            fro_last_list.emplace_back(i);
        }
    }
    vector<int> dense_iterval;//格式为两两一对，稠密开始和截至时间
    bool denseflag = false;
    for(int i=1; i < fro_last_list.size(); i++)
    {
        if(abs(fro_last_list[i]-fro_last_list[i-1]) < time_point_th)//此时判定为仍然处于稠密状态
        {
            if(!denseflag)
            {
                dense_iterval.emplace_back(fro_last_list[i-1]);//先存入稠密开始时间
                denseflag = true;
            }
        }else {
            if(denseflag)
            {
                dense_iterval.emplace_back(fro_last_list[i-1]);//再存入稠密结束时间
                denseflag = false;
            }
        }
    }
    if(denseflag)
        dense_iterval.emplace_back(fro_last_list[fro_last_list.size()-1]);
    return dense_iterval;
}
/*
*函数名称：SingleCar_CostTime
*函数功能：在没有前车阻碍的前提下，计算的单车的运行时间
*函数参数：单车s_car、 路数据m_road 顶点数、vex_time存放过每个路口的时间、（路口数）vexnum、全图最优路径存储矩阵
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
int SingleCar_CostTime(Car s_car, vector<Road> m_road)
{
    int length;
    int time_single=0;
    int S2 = 0;
    vector<int> vex_time;
    //vex_time.clear();
    for(uint i=0;i<s_car.road_path_index.size();i++)
    {
        int time_scost=0;
        int road_index = s_car.road_path_index[i];
        int next_index = s_car.road_path_index[i+1];
        length = m_road[road_index].length - S2;
        s_car.speed_now = min(s_car.speedmax, m_road[road_index].speedlim);//当前车辆在当前路上的实际速度

        if(length%s_car.speed_now == 0)//如果这段路到路口的距离正好在整数秒内可行驶完，则不用考虑过路口情况
        {
            time_scost = length/s_car.speed_now;//道路剩余的可行距离的消耗时间
            time_single +=time_scost;
            vex_time.emplace_back(time_single+1);//time_scost+1这个时间点车辆正好过路口
            S2 = 0;
        }else
        {
            int S1;
            S1 = length - (length/s_car.speed_now)*s_car.speed_now;//否则计算S1
            if(i == s_car.road_path_index.size()-1)//当到达路径最后一段路时
            {
                S2 = 0;
            }else{
                S2 = min(s_car.speedmax, m_road[next_index].speedlim) - S1;//否则S2=SV2-S1
                if(S2 < 0)
                    S2 = 0;
            }
            time_scost = length/s_car.speed_now + 1;//时间=速度对路长取整+1 ，才能到达下一个路口
            time_single +=time_scost;
            vex_time.emplace_back(time_single);//过路口时间点
        }
    }
    return time_single;
}


/*
*函数名称：Vex2Roadid
*函数功能：得到路口与路口对应路id的的矩阵idmat
*函数参数：道路数据 m_road
*函数返回值：idmat二维容器，idmat[起始点-1][终点-1]为对应路段的id
*Author:学学没完
*date：2019.3.14 0:39
*/
vector<vector<int>> Vex2Roadid(const vector<Road> &m_road)
{
    int start;
    int end;
    vector<vector<int>> idmat(m_road.size(),vector<int>(m_road.size(),0));
    for(uint count = 0; count < m_road.size(); count++)
    {
        start = m_road[count].start_ind;
        end = m_road[count].end_ind;
        idmat[start][end] = m_road[count].road_index;
        if(m_road[count].isDuplex==1)
        {
            idmat[end][start] = m_road[count].road_index;
        }
    }
    return idmat;
}

void V2Road_init(vector<Car> &m_car,vector<Road> &m_road, vector<vector<int>> V2Rmat)
{
    //idmat[起始点-1][终点-1]为对应路段的id
//    vector<vector<int>> V2Rmat;
//    V2Rmat = Vex2Roadid(m_road);
    for(uint i=0; i<m_car.size(); i++)
    {
        s_carV2Road(m_car[i],m_road,V2Rmat);
    }

}
void s_carV2Road(Car &s_car,vector<Road> &m_road, vector<vector<int>> V2Rmat)
{
    int sid = 0;
    int eid = 0;
    int vex_num = s_car.cross_path_index.size();
    s_car.road_path_index.clear();
    s_car.road_path_id.clear();
    for(int si=0;si<vex_num-1;si++)
    {
        if(si!=vex_num-2)
        {
            sid = s_car.cross_path_index[si];
            eid = s_car.cross_path_index[si+1];
            s_car.road_path_index.emplace_back(V2Rmat[sid][eid]);//赋值start_id到end_id+1的最优路径
            s_car.road_path_id.emplace_back(m_road[V2Rmat[sid][eid]].road_id);
        }
        else
        {
            sid = s_car.cross_path_index[si];
            eid = s_car.cross_path_index[si+1];
            s_car.road_path_index.emplace_back(V2Rmat[sid][eid]);//赋值start_id到end_id+1的最优路径
            s_car.road_path_id.emplace_back(m_road[V2Rmat[sid][eid]].road_id);
        }
    }
}
//对车的路径进行第一次规划
void pref_car_way_frirst(vector<Cross> &m_cross, vector<Road> &m_road, vector<Car> &op_car,vector<vector<int>> &ss_path_value, vector<vector<vector<int>>> &ss_path)
{
    //获取顶点数（即路口数）
    int vexnum = m_cross.size();

    //ss_path存放了所有点到点的最优路径的信息
    for (int j=0; j < vexnum; j++)//存放中间变量
    {
        vector<vector<int>> s_path;//中间变量
        vector<int> s_path_value;
        Graph_DG Map(vexnum);//输入参数就是路口数
        s_path = Map.Get_Vex_path(s_path_value,Map,m_road,j);//j是出发点，得到的s_path是j到图上任意点的最优路径路口集合
        ss_path.emplace_back(s_path);
        ss_path_value.emplace_back(s_path_value);
    }

    for(uint i=0; i<op_car.size(); i++)
    {
        op_car[i].cross_path_index = ss_path[op_car[i].start_ind][op_car[i].end_ind];//将所有的car中的cross_path_index成员变量放入路径的路口ind
    }
}

bool sortover(Car&p1, Car&p2)
{
    return (p1.car_over_cross_cout > p2.car_over_cross_cout);
}
//对车的路径进行二次规划
void pre_car_way_second(vector<Car> &op_car,vector<Cross> &m_cross, vector<Road> &m_road, vector<vector<vector<int>>> &ss_path)
{
    //解决道路过饱和的问题
    map<int,int> num_cross;//每个路口出现的次数
    map<int,vector<int>> over_cross_car;//每个路口过路的车
    vector<int> over_cross;
    //初始数量为0
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        num_cross[(*iter_cross).cross_index] = 0;
        over_cross_car[(*iter_cross).cross_index] = {};
    }
    //得到路口数目
    for(auto iter_car = op_car.begin(); iter_car != op_car.end(); iter_car++)
    {
        for(auto iter_cross = (*iter_car).cross_path_index.begin()+1; iter_cross != (*iter_car).cross_path_index.end()-1; iter_cross++)
        {
            num_cross[*iter_cross]++;
            over_cross_car[*iter_cross].emplace_back((*iter_car).car_index);//将车的索引放入
        }
    }
    vector<int> num_cross_sort;
    for(auto iter=num_cross.begin(); iter!=num_cross.end(); iter++)
    {
        if(iter->second != 0)
            num_cross_sort.emplace_back(iter->second);
    }
    sort(num_cross_sort.begin(), num_cross_sort.end());
    int N = num_cross_sort[num_cross_sort.size()*9/10];
    //得到饱和路口
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        if(num_cross[(*iter_cross).cross_index] > N )//num_cross_sort[num_cross_sort.size()*9/10]
        {
            over_cross.emplace_back((*iter_cross).cross_index);
        }
    }
    //首先需要重新生成地图
    vector<Road> again_road(m_road);
    vector<Cross> again_cross(m_cross);
    vector<int> del_road; //存放被删除的路

    //对道路重新规划 删除道路
    for(auto iter_again_road = again_road.begin(); iter_again_road != again_road.end(); )
    {
        auto iter1 = find(over_cross.begin(), over_cross.end(), (*iter_again_road).start_ind);
        auto iter2 = find(over_cross.begin(), over_cross.end(), (*iter_again_road).end_ind);
        if(iter1 != over_cross.end() || iter2 != over_cross.end())
        {
            del_road.emplace_back((*iter_again_road).road_index);//将过载路口的道路id放到del_road中
            iter_again_road = again_road.erase(iter_again_road);//将道路从路中删除
        }
        else
           iter_again_road++;
    }

    //删除路口
    for(auto iter_again_cross = again_cross.begin(); iter_again_cross != again_cross.end(); )
    {
        //删除路口
        auto iter3 = find(over_cross.begin(), over_cross.end(),(*iter_again_cross).cross_index);
        if(iter3 != over_cross.end())//如果该路口是过载路口，删除该路口
        {
            iter_again_cross = again_cross.erase(iter_again_cross);
        }
        else//如果不是，则判断该路口的道路是否在删除的道路里面
        {
            //对删除掉的路口应该标记为-1
            auto iter4 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[0]);
            auto iter5 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[1]);
            auto iter6 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[2]);
            auto iter7 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[3]);
            if(iter4 != del_road.end()) (*iter_again_cross).road_north_id = -1;
            if(iter5 != del_road.end()) (*iter_again_cross).road_east_id = -1;
            if(iter6 != del_road.end()) (*iter_again_cross).road_south_id = -1;
            if(iter7 != del_road.end()) (*iter_again_cross).road_west_id = -1;
            if((*iter_again_cross).road_north_id == -1 && (*iter_again_cross).road_east_id == -1 && (*iter_again_cross).road_south_id == -1 && (*iter_again_cross).road_west_id == -1)
            {
                over_cross.emplace_back((*iter_again_cross).cross_index);//如果路口的道路全部被删除了，则将该路口放到过载的路口中，同时删除该路口
                iter_again_cross = again_cross.erase(iter_again_cross);
            }
            else
            {
               iter_again_cross++;
            }
        }
    }
    cout <<"新路的数目:"<< again_road.size() << "新路口的数目:" << again_cross.size() << endl;

    int again_vexnum;
    again_vexnum = m_cross.size();
    //Get_Vexnum(again_road, again_vexnum);
    cout << "agian_vexnum:" << again_vexnum << endl;
    //ss_path存放了所有点到点的最优路径的信息
    vector<vector<vector<int>>> again_ss_path;//[start_id-1][end_id-1][i]
    vector<vector<int>> again_ss_path_value;
    for (int j=0; j < again_vexnum; j++)//存放中间变量
    {
        vector<vector<int>> again_s_path;//中间变量
        vector<int> again_s_path_value;
        Graph_DG Map(again_vexnum);//输入参数就是路口数
        again_s_path = Map.Get_Vex_path(again_s_path_value,Map,again_road,j);//j是出发点，得到的s_path是j到图上任意点的最优路径路口集合
        again_ss_path.emplace_back(again_s_path);
        again_ss_path_value.emplace_back(again_s_path_value);
    }

    set <int> change_car_ind_d;
    //对过饱和的路口的车进行重新路线规划 先得到需要更改路线车的ind
    cout << over_cross_car.size() << endl;
    for(auto iters=over_cross_car.begin(); iters!=over_cross_car.end(); iters++)
    {
        if((iters->second).size()< N)
        {
            over_cross_car.erase(iters);
        }else {
            for(int j = 0; j < (iters->second).size() - N; j++)
            {
                change_car_ind_d.insert((iters->second)[j]);
            }
        }
    }
    vector<Car> que_car;
    for(auto iter = change_car_ind_d.begin(); iter != change_car_ind_d.end(); iter++)
    {
        for(auto element : over_cross_car)
        {
            for(int j = 0; j < element.second.size(); j++)
            {
                if(*iter == element.second[j])
                {
                    op_car[*iter].car_over_cross_cout++;
                    break;
                }
            }
        }
        que_car.emplace_back(op_car[*iter]);
    }
    sort(que_car.begin(),que_car.end(),sortover);
    set <int> change_car_ind;
    for(auto iterq=que_car.begin(); iterq!=que_car.end()-que_car.size()*2/3; iterq++)
    {
        change_car_ind.insert(iterq->car_index);
    }

    int num_change_car = 0;
    for(auto iter = change_car_ind.begin(); iter!=change_car_ind.end();iter++)
    {
        //先判断路的起点和终点有没有被删除 没有被删除才能重新规划
        auto iter8 = find(over_cross.begin(), over_cross.end(), op_car[*iter].start_ind);
        auto iter9 = find(over_cross.begin(), over_cross.end(), op_car[*iter].end_ind);
        if(iter8 == over_cross.end() && iter9 == over_cross.end())
        {
            //判断出发点和终点点之间是联通的
            cout << "重新规划的车辆" << endl;
            cout << op_car[*iter].car_index << endl;

            if(again_ss_path_value[op_car[*iter].start_ind][op_car[*iter].end_ind] > 0 && again_ss_path_value[op_car[*iter].start_ind][op_car[*iter].end_ind] < 2000000)
            {
                //如果车的路径改变，同时对ss_path进行更新，便于后期排序
                ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind] = again_ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind];
                //对车的路径进行更新
                op_car[*iter].cross_path_index = again_ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind];
                num_change_car++;
            }
        }
    }
    //V2Road_init(op_car,m_road,V2Rmat);//初始化所有的car中的road_path_index和road_path_id
    cout << "已重新规划路线的车的数目为:" << num_change_car << endl;
}


//自定义排序函数
bool sortarrange(const Car &p1, const Car &p2)
{
    if(p1.priority != p2.priority) return p1.priority > p2.priority;
    if(p1.speedmax != p2.speedmax) return p1.speedmax > p2.speedmax;
    return (p1.cost_time < p2.cost_time);//升序排序
}



/*
*函数名称：print_answertxt
*函数功能：输出answer.txt
*函数参数：显而易见
*函数返回值：无
*Author:学学没完
*date：2019.3.15 23:47
*
* 修改日志：适应当前版本，变成纯输出函数
* 修改时间：2019.3.16 22:20
*/
void print_answertxt(vector<vector<int>> V2Rmat,int car_batch,int go_time,const vector<Car> &m_car,const vector<Road> m_road,string answerPath)
{
    //初始化输出txt
    string filepath = answerPath;
    ofstream put;
    put.open(filepath.c_str(), ios::app);
    //for(vector<int>::iterator iter=car_batch.begin(); iter!=car_batch.end(); iter++)
    {
        put<<"("<<m_car[car_batch].car_id<<","<<go_time<<",";
        int sid = 0;
        int eid = 0;
        int vex_num = m_car[car_batch].cross_path_index.size();
        for(int si=0;si<vex_num-1;si++)
        {
            if(si!=vex_num-2)
            {
                sid = m_car[car_batch].cross_path_index[si];
                eid = m_car[car_batch].cross_path_index[si+1];
                put<<m_road[V2Rmat[sid][eid]].road_id<<",";//输出start_id到end_id+1的最优路径
            }
            else
            {
                sid = m_car[car_batch].cross_path_index[si];
                eid = m_car[car_batch].cross_path_index[si+1];
                put<<m_road[V2Rmat[sid][eid]].road_id;
            }

        }
        put<<")"<<endl;
    }

}

/*
*函数名称：pri_car_GetOverCross
*函数功能：通过预置车辆的路径获得过载路口
*函数参数：略
*函数返回值：过载路口over_cross
*Author:学学没完
*date：2019.4.10 01:07
*/
vector<int> pri_car_GetOverCross(vector<Cross> &m_cross, vector<presetCar> &m_pcar)
{
    //解决道路过饱和的问题
    map<int,int> num_cross;//每个路口出现的次数
    vector<int> over_cross;
    //初始数量为0
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        num_cross[(*iter_cross).cross_index] = 0;
    }
    //得到路口数目
    for(auto iter_car = m_pcar.begin(); iter_car != m_pcar.end(); iter_car++)
    {
        for(auto iter_cross = (*iter_car).cross_path_PA.begin()+1; iter_cross != (*iter_car).cross_path_PA.end()-1; iter_cross++)
        {
            num_cross[*iter_cross]++;
        }
    }
    //确定饱和阈值
    vector<int> num_cross_sort;
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        num_cross_sort.emplace_back(num_cross[(*iter_cross).cross_index]);
    }
    sort(num_cross_sort.begin(),num_cross_sort.end());
    int th_N = num_cross_sort[num_cross_sort.size()*9/10];
    //得到饱和路口
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        if(num_cross[(*iter_cross).cross_index] > th_N)
        {
            over_cross.emplace_back((*iter_cross).cross_index);
        }
    }
    return over_cross;
}

/*
*函数名称：Gen_new_pri_path
*函数功能：已知预置车辆过载路口over_cross，来剔除这些路口，并重新生成图，
*函数参数：again_ss_path新图路径，again_ss_path_value新图路径长度值
*函数返回值：无
*Author:学学没完
*date：2019.4.10 01:07
*/
void Gen_new_pri_path(vector<Cross> &m_cross, vector<Road> &m_road,vector<int> over_cross,vector<vector<vector<int>>> &again_ss_path, vector<vector<int>> &again_ss_path_value)
{
    //首先需要重新生成地图
    vector<Road> again_road(m_road);
    vector<Cross> again_cross(m_cross);
    vector<int> del_road; //存放被删除的路

    //对道路重新规划 删除道路
    for(auto iter_again_road = again_road.begin(); iter_again_road != again_road.end(); )
    {
        auto iter1 = find(over_cross.begin(), over_cross.end(), (*iter_again_road).start_ind);
        auto iter2 = find(over_cross.begin(), over_cross.end(), (*iter_again_road).end_ind);
        if(iter1 != over_cross.end() || iter2 != over_cross.end())
        {
            del_road.emplace_back((*iter_again_road).road_index);//将过载路口的道路id放到del_road中
            iter_again_road = again_road.erase(iter_again_road);//将道路从路中删除
        }
        else
           iter_again_road++;
    }

    //删除路口
    for(auto iter_again_cross = again_cross.begin(); iter_again_cross != again_cross.end(); )
    {
        //删除路口
        auto iter3 = find(over_cross.begin(), over_cross.end(),(*iter_again_cross).cross_index);
        if(iter3 != over_cross.end())//如果该路口是过载路口，删除该路口
        {
            iter_again_cross = again_cross.erase(iter_again_cross);
        }
        else//如果不是，则判断该路口的道路是否在删除的道路里面
        {
            //对删除掉的路口应该标记为-1
            auto iter4 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[0]);
            auto iter5 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[1]);
            auto iter6 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[2]);
            auto iter7 = find(del_road.begin(), del_road.end(), (*iter_again_cross).cross_road_index[3]);
            if(iter4 != del_road.end()) (*iter_again_cross).road_north_id = -1;
            if(iter5 != del_road.end()) (*iter_again_cross).road_east_id = -1;
            if(iter6 != del_road.end()) (*iter_again_cross).road_south_id = -1;
            if(iter7 != del_road.end()) (*iter_again_cross).road_west_id = -1;
            if((*iter_again_cross).road_north_id == -1 && (*iter_again_cross).road_east_id == -1 && (*iter_again_cross).road_south_id == -1 && (*iter_again_cross).road_west_id == -1)
            {
                over_cross.emplace_back((*iter_again_cross).cross_index);//如果路口的道路全部被删除了，则将该路口放到过载的路口中，同时删除该路口
                iter_again_cross = again_cross.erase(iter_again_cross);
            }
            else
            {
               iter_again_cross++;
            }
        }
    }
    cout <<"新路的数目:"<< again_road.size() << "新路口的数目:" << again_cross.size() << endl;

    int again_vexnum;
    again_vexnum = m_cross.size();
    cout << "agian_vexnum:" << again_vexnum << endl;
    for (int j=0; j < again_vexnum; j++)//存放中间变量
    {
        vector<vector<int>> again_s_path;//中间变量
        vector<int> again_s_path_value;
        Graph_DG Map(again_vexnum);//输入参数就是路口数
        again_s_path = Map.Get_Vex_path(again_s_path_value,Map,again_road,j);//j是出发点，得到的s_path是j到图上任意点的最优路径路口集合
        again_ss_path.emplace_back(again_s_path);
        again_ss_path_value.emplace_back(again_s_path_value);
    }
}

/*
*函数名称：Change_singleCar_path
*函数功能：已知被删除的路口,新生成的路径和值,改变该车的cross_path_index(即路径)
*函数参数：被删除的路口over_cross,新生成的路径和值again_ss_path、again_ss_path_value
*函数返回值：无
*Author:学学没完
*date：2019.4.10 01:15
*/
void Change_singleCar_path(vector<int> over_cross ,Car &s_car, vector<vector<vector<int>>> again_ss_path, vector<vector<int>> again_ss_path_value)
{
    //先判断路的起点和终点有没有被删除 没有被删除才能重新规划
    auto iter8 = find(over_cross.begin(), over_cross.end(), s_car.start_ind);
    auto iter9 = find(over_cross.begin(), over_cross.end(), s_car.end_ind);
    if(iter8 == over_cross.end() && iter9 == over_cross.end())
    {
        if(again_ss_path_value[s_car.start_ind][s_car.end_ind] > 0 && again_ss_path_value[s_car.start_ind][s_car.end_ind] < 2000000)
        {
            //对车的路径进行更新
            s_car.cross_path_index = again_ss_path[s_car.start_ind][s_car.end_ind];
        }
    }
}
void Change_singleCar_path(Car &s_car, vector<vector<vector<int>>> again_ss_path)
{
    //对车的路径进行更新
    s_car.cross_path_index = again_ss_path[s_car.start_ind][s_car.end_ind];
}

/*
*函数名称：pri_car_deal
*函数功能：遍历所有非预置车辆，并将可更改其路径（预置过载路口删除后）的车处理排序后返回  效果不佳，已弃用
*函数参数：被删除的路口over_cross,新生成的路径和值again_ss_path、again_ss_path_value
*函数返回值：发车索引
*Author:学学没完
*date：2019.4.10 01:17
*/
vector<int> pri_car_deal(vector<Car> &m_car,vector<Cross> &m_cross, vector<Road> &m_road,vector<presetCar> &m_pcar, signed int N, vector<vector<vector<int>>> &ss_path)
{
    //解决道路过饱和的问题
    vector<int> over_cross;
    over_cross = pri_car_GetOverCross(m_cross,m_pcar);
    vector<vector<vector<int>>> again_ss_path;
    vector<vector<int>> again_ss_path_value;
    Gen_new_pri_path(m_cross,m_road,over_cross,again_ss_path,again_ss_path_value);

    vector<Car> sur_car;//优化路径的车，他们的起始点和终止点都不在过载路口
    vector<vector<vector<int>>> ss_path_temp = ss_path;//存放临时路径
    int num_change_car = 0;
    for(auto iter = m_car.begin(); iter!=m_car.end();iter++)
    {
        //先判断路的起点和终点有没有被删除 没有被删除才能重新规划
        auto iter8 = find(over_cross.begin(), over_cross.end(), iter->start_ind);
        auto iter9 = find(over_cross.begin(), over_cross.end(), iter->end_ind);
        if(iter8 == over_cross.end() && iter9 == over_cross.end())
        {
            if(again_ss_path_value[iter->start_ind][iter->end_ind] > 0 && again_ss_path_value[iter->start_ind][iter->end_ind] < 2000000)
            {
                sur_car.emplace_back(*iter);
                //如果车的路径改变，同时对ss_path进行更新，便于后期排序
                ss_path_temp[iter->start_ind][iter->end_ind] = again_ss_path[iter->start_ind][iter->end_ind];
                //对车的路径进行更新
                iter->cross_path_index = again_ss_path[iter->start_ind][iter->end_ind];
                num_change_car++;
            }
        }
    }
    //V2Road_init(m_car,m_road,V2Rmat);//初始化所有的car中的road_path_index和road_path_id
    return pri_car_deal_part(m_cross,sur_car);
}

/*
*函数名称：pri_car_deal_part
*函数功能：pri_car_deal的部分，对更改过路径的车进行排序，得到发车序列
*函数参数：更改过路径的车sur_car
*函数返回值：发车索引
*Author:学学没完
*date：2019.4.10 01:23
*/
vector<int> pri_car_deal_part(vector<Cross> m_cross,vector<Car> &sur_car)
{
    map<int,vector<Car>> arrange_car_map;
    vector<int> cross_ind; //存放路口的index
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        arrange_car_map[(*iter_cross).cross_index] = {};
        cross_ind.emplace_back((*iter_cross).cross_index);
    }
    for(auto iter_car = sur_car.begin(); iter_car != sur_car.end(); iter_car++)
    {
        arrange_car_map[(*iter_car).start_ind].emplace_back(*iter_car);
    }
    //此处对车进行排序
    for(auto iter_cross_id = cross_ind.begin(); iter_cross_id != cross_ind.end(); iter_cross_id++)
    {
        sort(arrange_car_map[*iter_cross_id].begin(),arrange_car_map[*iter_cross_id].end(),sortarrange);
    }

    //把发车顺序写成一个list
    vector<int> car_wait_arrange_pri;//等待出发的车
    for(int i = 0; i < sur_car.size();)
    {
        for(auto iter_cross_id = cross_ind.begin(); iter_cross_id != cross_ind.end(); iter_cross_id++)
        {
            if(arrange_car_map[*iter_cross_id].size() != 0)
            {
                auto it_car = arrange_car_map[*iter_cross_id].begin();
                car_wait_arrange_pri.emplace_back((*it_car).car_index);
                i++;//放进一辆车，i加1
                arrange_car_map[*iter_cross_id].erase(it_car);
            }

        }
    }
    return car_wait_arrange_pri;
}

void Arrange_pri(vector<Car> &opp_car,vector<int> &gotime,vector<int> &car_arrange_index,vector<Car> &op_car,vector<Road> m_road,vector<int> car_wait_arrange_pri,string answerPath, int time_interval)
{
    int time_now = 1;
    while(time_now<760)
    {
        int pri_th = 600;
        int batch_car_num = 0;
        int car_batch;
        int start_time;
        for(vector<int>::iterator iter=car_wait_arrange_pri.begin(); iter!=car_wait_arrange_pri.end();)
        {
            if(batch_car_num <= pri_th)
            {
                car_batch= *iter;//将可出发的车放到car_batch
                car_arrange_index.emplace_back(*iter);
                iter = car_wait_arrange_pri.erase(iter);//将该车从剩余等待车的数组里清除出去
                batch_car_num += 1;//计数批次车辆数目
                if(op_car[car_batch].plantime > time_now)
                {
                    start_time = op_car[car_batch].plantime;
                }else{
                    start_time = time_now;
                }
                gotime.emplace_back(start_time);
                op_car[car_batch].gotime = start_time;
                //print_answertxt(car_batch,start_time,op_car,m_road,answerPath);
            }else
            {
                iter++;
                break;
            }

        }
        time_now = time_now+time_interval;
    }
    for(auto iter=car_arrange_index.begin();iter!=car_arrange_index.end();iter++)
    {
        for(auto itera=opp_car.begin(); itera!=opp_car.end();)
        {
            if(itera->car_index == *iter)
                itera = opp_car.erase(itera);
            else
                itera++;
        }
    }
   // return car_wait_arrange_pri;
}

void Arrange_sur(vector<vector<vector<int>>> ss_path_p,vector<int> dense_iterval,vector<vector<int>> V2Rmat,vector<int> over_cross, vector<vector<vector<int>>> again_ss_path, vector<vector<int>> again_ss_path_value, vector<int> &gotime, vector<int> &car_arrange_index,vector<Car> &op_car,vector<Road> m_road,vector<Cross> m_cross,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g)
{
    //先构造不同map
    map<int,vector<Car>> arrange_car_map;
    vector<int> cross_ind; //存放路口的index
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        arrange_car_map[(*iter_cross).cross_index] = {};
        cross_ind.emplace_back((*iter_cross).cross_index);
    }
    for(auto iter_car = op_car.begin(); iter_car != op_car.end(); iter_car++)
    {
        arrange_car_map[(*iter_car).start_ind].emplace_back(*iter_car);
    }

    //此处对车进行排序
    for(auto iter_cross_id = cross_ind.begin(); iter_cross_id != cross_ind.end(); iter_cross_id++)
    {
        sort(arrange_car_map[*iter_cross_id].begin(),arrange_car_map[*iter_cross_id].end(),sortarrange);
    }

    //把发车顺序写成一个list
    vector<int> car_wait_arrange;//等待出发的车
    for(int i = 0; i < op_car.size();)
    {
        for(auto iter_cross_id = cross_ind.begin(); iter_cross_id != cross_ind.end(); iter_cross_id++)
        {
            if(arrange_car_map[*iter_cross_id].size() != 0)
            {
                auto it_car = arrange_car_map[*iter_cross_id].begin();
                car_wait_arrange.emplace_back((*it_car).car_index);
                i++;//放进一辆车，i加1
                arrange_car_map[*iter_cross_id].erase(it_car);
            }
        }
    }
    Arrange_sur_part(ss_path_p,dense_iterval,V2Rmat,over_cross,again_ss_path,again_ss_path_value,gotime,car_arrange_index,op_car,car_wait_arrange,m_road, answerPath, caronroad_threshold, time_interval,para_c,para_d,para_e,para_f,para_g);
}

void Arrange_sur_part(vector<vector<vector<int>>> ss_path_p,vector<int> dense_iterval,vector<vector<int>> V2Rmat,vector<int> over_cross, vector<vector<vector<int>>> again_ss_path, vector<vector<int>> again_ss_path_value, vector<int> &gotime,vector<int> &car_arrange_index, vector<Car> &op_car,vector<int> car_wait,vector<Road> m_road,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g)
{

    int time_now = 1;
    int start_time = 1;
    int th_temp = caronroad_threshold;
    string filepath = answerPath;
    ofstream put1;
    put1.open(filepath.c_str(), ios::app);
    put1<<"#(car_id  gotime  path)"<<endl;
    int DI_index=0;
    bool dense_mode = false;
    while(!car_wait.empty())
    {

        if(DI_index < dense_iterval.size())
        {
            if(dense_mode == false)
            {
                if(dense_iterval[DI_index] < (time_now+20))//当当前的时间点+10s过了该时间,提前进入
                {
                    dense_mode = true;
                    DI_index++;
                }
            }else
            {
                if(dense_iterval[DI_index] < (time_now-20))//当当前的时间点-10s过了该时间，延后退出
                {
                    dense_mode = false;
                    DI_index++;
                }
            }
        }
        int batch_car_num = 0;
        int car_batch;
        if(tu==1)
        {
            if(dense_mode)
            {
                caronroad_threshold = 400;
            }else if(time_now<1100)
            {
                caronroad_threshold = th_temp+350;
            }else{
                caronroad_threshold = th_temp-100;
            }
        }else
        {
            if(dense_mode)
            {
                caronroad_threshold = para_c;
            }else if(time_now<para_d)
            {
                caronroad_threshold = th_temp+para_e;
            }else{
                caronroad_threshold = th_temp-para_f;
            }
        }

        for(vector<int>::iterator iter=car_wait.begin(); iter!=car_wait.end();)
        {
            if(batch_car_num <= caronroad_threshold)
            {
                car_batch = *iter;//将可出发的车放到car_batch
                car_arrange_index.emplace_back(*iter);
                iter = car_wait.erase(iter);//将该车从剩余等待车的数组里清除出去
                batch_car_num += 1;//计数批次车辆数目
                if(op_car[car_batch].plantime > time_now)
                {
                    start_time = op_car[car_batch].plantime;
                }else{
                    start_time = time_now;
                }
                gotime.emplace_back(start_time);
                op_car[car_batch].gotime = start_time;
                if(dense_mode)
                {
                    Change_singleCar_path(over_cross ,op_car[car_batch],again_ss_path,again_ss_path_value);
                    //s_carV2Road(op_car[car_batch],m_road,V2Rmat);
                }
                if(time_now > para_g)
                {
//                    Change_singleCar_path(op_car[car_batch], ss_path_p);
                }
                print_answertxt(V2Rmat,car_batch,start_time,op_car,m_road,answerPath);
            }else
            {
                iter++;
                break;
            }

        }
        time_now = time_now+time_interval;
        cout<<"car_wait.size():"<<car_wait.size()<<endl;
    }
}



void Arrange1(vector<int> &gotime,vector<int> &car_arrange_index, vector<Car> &m_car,vector<int> car_wait,vector<Road> m_road,string answerPath, int caronroad_threshold, int time_interval)
{
    int time_now = 1;
    int start_time = 1;
    string filepath = answerPath;
    ofstream put1;
    put1.open(filepath.c_str(), ios::app);
    put1<<"#(car_id  gotime  path)"<<endl;
    while(!car_wait.empty())
    {
        int batch_car_num = 0;
        int car_batch;

        for(vector<int>::iterator iter=car_wait.begin(); iter!=car_wait.end();)
        {
            if(batch_car_num <= caronroad_threshold)
            {
                car_batch = *iter;//将可出发的车放到car_batch
                car_arrange_index.emplace_back(*iter);
                iter = car_wait.erase(iter);//将该车从剩余等待车的数组里清除出去
                batch_car_num += 1;//计数批次车辆数目
                if(m_car[car_batch].plantime > time_now)
                {
                    start_time = m_car[car_batch].plantime;
                }else{
                    start_time = time_now;
                }

                gotime.emplace_back(start_time);
                m_car[car_batch].gotime = start_time;
               // print_answertxt(car_batch,start_time,m_car,m_road,answerPath);
            }else
            {
                iter++;
                break;
            }

        }
        time_now = time_now+time_interval;
        cout<<"car_wait.size():"<<car_wait.size()<<endl;
    }
}

/*
安排发车算法3 继续对车进行稀疏 按不同路口进行发车 不同路口再按照速度和消耗时间对车辆进行排序
*/
void Arrange2(vector<int> &gotime, vector<presetCar> m_pcar,vector<int> &car_arrange_index,vector<Car> &op_car,vector<Road> m_road,vector<Cross> m_cross,string answerPath, int caronroad_threshold, int time_interval)
{
    //先构造不同map
    map<int,vector<Car>> arrange_car_map;
    vector<int> cross_ind; //存放路口的index
    for(auto iter_cross = m_cross.begin(); iter_cross != m_cross.end(); iter_cross++)
    {
        arrange_car_map[(*iter_cross).cross_index] = {};
        cross_ind.emplace_back((*iter_cross).cross_index);
    }
    for(auto iter_car = op_car.begin(); iter_car != op_car.end(); iter_car++)
    {
        arrange_car_map[(*iter_car).start_ind].emplace_back(*iter_car);
    }
    //此处对车进行排序
    for(auto iter_cross_id = cross_ind.begin(); iter_cross_id != cross_ind.end(); iter_cross_id++)
    {
        sort(arrange_car_map[*iter_cross_id].begin(),arrange_car_map[*iter_cross_id].end(),sortarrange);
    }

    //把发车顺序写成一个list
    vector<int> car_wait_arrange;//等待出发的车
    int car_num = op_car.size();
    for(int i = 0; i < car_num;)
    {
        for(auto iter_cross_id = cross_ind.begin(); iter_cross_id != cross_ind.end(); iter_cross_id++)
        {
            if(arrange_car_map[*iter_cross_id].size() != 0)
            {
                auto it_car = arrange_car_map[*iter_cross_id].begin();
                car_wait_arrange.emplace_back((*it_car).car_index);
                i++;//放进一辆车，i加1
                arrange_car_map[*iter_cross_id].erase(it_car);
            }

        }
    }
    Arrange1(gotime,car_arrange_index,op_car,car_wait_arrange,m_road, answerPath, caronroad_threshold, time_interval);
}

