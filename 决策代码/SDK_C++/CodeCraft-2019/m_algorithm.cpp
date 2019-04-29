#include "m_algorithm.h"
#include "graph.h"
#include<stdlib.h>

//获取车的在路上跑时长
void Init_Cost_Time(vector<Car> &op_car,vector<Road> m_road)
{
    for(uint i=0; i<op_car.size(); i++)
    {
        op_car[i].cost_time = SingleCar_CostTime(op_car[i], m_road);
    }
}

bool sortdeal(Car&p1, Car&p2)
{
    if(p1.priority != p2.priority) return p1.priority > p2.priority;
    if(p1.speedmax != p2.speedmax) return p1.speedmax > p2.speedmax;
    return (p1.cost_time < p2.cost_time);//升序排序
}
//通过对costtime的排序，将花费时间短的归为需要处理的车，
vector<Car> Get_deal_car(vector<Car> op_car)
{
    sort(op_car.begin(),op_car.end(),sortdeal);
    vector<Car> deal_car(op_car.begin(),op_car.begin()+op_car.size()*2/3);
    return deal_car;
}

void pre_car_way_second_v2(vector<Car> &op_car,vector<Car> &m_car,vector<Cross> &m_cross, vector<Road> &m_road, vector<vector<vector<int>>> &ss_path)
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
                //op_car[*iter].cross_path_index = again_ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind];
                m_car[op_car[*iter].op_car_index].cross_path_index = again_ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind];
                num_change_car++;
            }
        }
    }
    //V2Road_init(op_car,m_road,V2Rmat);//初始化所有的car中的road_path_index和road_path_id
    cout << "已重新规划路线的车的数目为:" << num_change_car << endl;
}
