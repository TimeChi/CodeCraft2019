#include "referee_system.h"
#include "m_algorithm.h"
#include "graph.h"
int tu =0;
#define KK 0
vector<Car> pcar_way_second(map<int, int> road_map,vector<Car> &m_car,vector<int> over_cross,vector<presetCar> &m_pcar,vector<Cross> &m_cross, vector<Road> &m_road,vector<vector<int>> &V2Rmat);
void Scheme_One(vector<Car> &op_car,vector<Road> &m_road,vector<Cross> &m_cross,vector<presetCar> &m_pcar,vector<vector<int>> &V2Rmat,vector<int> &dense_iterval,vector<int> &gotime, vector<int> &car_arrange_index,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g);
void Scheme_Two(vector<Car> &m_car,map<int, int> road_map,vector<Car> &op_car,vector<Road> &m_road,vector<Cross> &m_cross,vector<presetCar> &m_pcar,vector<vector<int>> &V2Rmat,vector<int> &dense_iterval,vector<int> &gotime, vector<int> &car_arrange_index,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g);

int main(int argc, char *argv[])
{
    std::cout << "Begin" << std::endl;
	
	if(argc < 6){
		std::cout << "please input args: carPath, roadPath, crossPath, answerPath" << std::endl;
		exit(1);
	}
	
	std::string carPath(argv[1]);
	std::string roadPath(argv[2]);
	std::string crossPath(argv[3]);
	std::string presetAnswerPath(argv[4]);
	std::string answerPath(argv[5]);
	

	std::cout << "carPath is " << carPath << std::endl;
	std::cout << "roadPath is " << roadPath << std::endl;
	std::cout << "crossPath is " << crossPath << std::endl;
	std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
	std::cout << "answerPath is " << answerPath << std::endl;

    std::string paramPath("../config/paramPath.txt");
    vector<int> Para_A;
    vector<int> Para_B;
    vector<int> Para_C;
    vector<int> Para_D;
    vector<int> Para_E;
    vector<int> Para_F;
    vector<int> Para_G;
    ifstream infileparam(paramPath);
    myReadparam(infileparam,Para_A,Para_B,Para_C,Para_D,Para_E,Para_F,Para_G);
    int para_a=Para_A[KK],para_b=Para_B[KK],para_c=Para_C[KK],para_d=Para_D[KK],para_e=Para_E[KK],para_f=Para_F[KK],para_g=Para_G[KK];
//计时，查看系统运行时间
    time_t begin,end;
    double ret;
    begin = clock();
    //数据初始化
    vector<Car> m_car;
    vector<Road> m_road;
    vector<Cross> m_cross;
    vector<presetCar> m_pcar;
    map<int, int> cross_map;
    map<int, int> road_map;
    map<int, int> car_map;
    Data_init(m_car,m_road,m_cross,m_pcar,carPath,roadPath,crossPath,presetAnswerPath);
    Init_all(m_cross,m_road,m_car,cross_map,road_map,car_map);
    //稀疏稠密
    vector<int> dense_iterval = Get_dense_iterval(m_pcar);
    vector<Car> op_car = m_car;//所有的算法操作的车都只在op_car中
    Car_map_one(op_car,cross_map);
    //这段程序处理完后，m_car里的预置车辆的cross_path_index全部被初始化;且op_car中只剩余非预置车辆
    for(uint i=0;i<m_pcar.size();i++)
    {
        auto iter = find_if(op_car.begin(),op_car.end(),vector_finder(m_pcar[i].car_id_PA));
        PARoad2Cross(*iter,m_road,m_pcar[i],road_map);
        op_car.erase(iter);
    }
    Car_map_two(op_car,car_map);
    vector<vector<int>> V2Rmat;
    V2Rmat = Vex2Roadid(m_road);

    //对饱和路口作处理，对路口进行二次规划
    int caronroad_threshold = 0;//caronroad_threshold_arr[KK];
    int time_interval = 0;//time_interval_arr[KK];
    if(m_car[0].car_id==37819)
    {
       caronroad_threshold = 1000;
       time_interval = 20;
       tu = 1;
    }else{
       caronroad_threshold = para_a;
       time_interval = para_b;
       tu = 2;
    }
    vector<int> gotime;
    vector<int> car_arrange_index;
   // Scheme_One(op_car,m_road,m_cross,m_pcar,V2Rmat,dense_iterval,gotime,car_arrange_index,answerPath,caronroad_threshold,time_interval,para_c,para_d,para_e,para_f,para_g);

Scheme_Two(m_car,road_map,op_car,m_road,m_cross,m_pcar,V2Rmat,dense_iterval,gotime,car_arrange_index,answerPath,caronroad_threshold,time_interval,para_c,para_d,para_e, para_f,para_g);
    
    //测试代码运行时间
    end = clock();
    ret=double(end-begin)/CLOCKS_PER_SEC;
    cout<<"runtime:   "<<ret<<" s"<<endl;

    return 0;
}

//计划一：对全部非预置车辆进行一次规划二次规划，基础版
void Scheme_One(vector<Car> &op_car,vector<Road> &m_road,vector<Cross> &m_cross,vector<presetCar> &m_pcar,vector<vector<int>> &V2Rmat,vector<int> &dense_iterval,vector<int> &gotime, vector<int> &car_arrange_index,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g)
{
    vector<vector<int>> ss_path_value_p;
    vector<vector<vector<int>>> ss_path_p;
    pref_car_way_frirst(m_cross,m_road,op_car,ss_path_value_p,ss_path_p);
    pre_car_way_second(op_car,m_cross, m_road,ss_path_p);
    Init_Cost_Time(op_car,m_road);

    vector<vector<vector<int>>> again_ss_path;
    vector<vector<int>> again_ss_path_value;
    vector<int> over_cross;
    over_cross = pri_car_GetOverCross(m_cross,m_pcar);
    Gen_new_pri_path(m_cross,m_road,over_cross,again_ss_path,again_ss_path_value);
    Arrange_sur(ss_path_p,dense_iterval,V2Rmat,over_cross,again_ss_path,again_ss_path_value,gotime,car_arrange_index,op_car,m_road,m_cross,answerPath,caronroad_threshold,time_interval,para_c,para_d,para_e,para_f,para_g);
    V2Road_init(op_car,m_road,V2Rmat);//初始化所有的car中的road_path_index和road_path_id
}

void Car_map_six(vector<presetCar> &p_car, map<int, int> &pcar_map)
{
    for(uint i=0; i<p_car.size(); i++)
    {
        pcar_map.insert(pair<int, int>(p_car[i].car_id_PA, p_car[i].car_index));
    }
}
void Scheme_Two(vector<Car> &m_car,map<int, int> road_map,vector<Car> &op_car,vector<Road> &m_road,vector<Cross> &m_cross,vector<presetCar> &m_pcar,vector<vector<int>> &V2Rmat,vector<int> &dense_iterval,vector<int> &gotime, vector<int> &car_arrange_index,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g)
{
    vector<int> over_cross;
    over_cross = pri_car_GetOverCross(m_cross,m_pcar);//获得预置车过载路口
    vector<Car> already_change_pcar = pcar_way_second(road_map,m_car,over_cross,m_pcar,m_cross,m_road,V2Rmat);
    map<int,int> pcar_map;
    Car_map_six(m_pcar,pcar_map);
    vector<presetCar> m_ppcar;
    for(auto iters=already_change_pcar.begin();iters!=already_change_pcar.end();iters++)
    {
        print_answertxt(V2Rmat,iters->car_index,iters->gotime,m_car,m_road,answerPath);
        auto iterb = pcar_map.find(iters->car_id);
        m_ppcar.emplace_back(m_pcar[iterb->second]);
    }
//    for(auto iter1=m_ppcar.begin();iter1!=m_ppcar.end();iter1++)
//    {
//        auto iter2 = find_if(m_pcar.begin(),m_pcar.end(),pfun1);
//    }
    vector<vector<int>> ss_path_value_p;
    vector<vector<vector<int>>> ss_path_p;
    pref_car_way_frirst(m_cross,m_road,op_car,ss_path_value_p,ss_path_p);
    pre_car_way_second(op_car,m_cross, m_road,ss_path_p);
    Init_Cost_Time(op_car,m_road);

    vector<vector<vector<int>>> again_ss_path;
    vector<vector<int>> again_ss_path_value;

    Gen_new_pri_path(m_cross,m_road,over_cross,again_ss_path,again_ss_path_value);
    Arrange_sur(ss_path_p,dense_iterval,V2Rmat,over_cross,again_ss_path,again_ss_path_value,gotime,car_arrange_index,op_car,m_road,m_cross,answerPath,caronroad_threshold,time_interval,para_c,para_d,para_e,para_f,para_g);
    V2Road_init(op_car,m_road,V2Rmat);//初始化所有的car中的road_path_index和road_path_id
}

void Car_map_five(vector<Car> &m_car, map<int, int> &car_map)
{
    for(uint i=0; i<m_car.size(); i++)
    {
        car_map.insert(pair<int, int>(m_car[i].car_id, m_car[i].car_index));
    }
}
bool m_pfun(Car &p1, Car &p2)
{
    if(p1.pcar_overnum!=p2.pcar_overnum) return p1.pcar_overnum>p2.pcar_overnum;
    if(p1.priority!=p2.priority) return p1.priority>p2.priority;
    if(p1.speedmax!=p2.speedmax) return p1.speedmax>p2.speedmax;

}
//只是对预置车辆的更改路径
vector<Car> pcar_way_second(map<int, int> road_map,vector<Car> &m_car,vector<int> over_cross,vector<presetCar> &m_pcar,vector<Cross> &m_cross, vector<Road> &m_road,vector<vector<int>> &V2Rmat)
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
    vector<Car> pcar;
    map<int, int> car_map;
    Car_map_five(m_car, car_map);
    for(auto iter=m_pcar.begin();iter!=m_pcar.end();iter++)
    {
        auto itera = car_map.find(iter->car_id_PA);
        m_car[itera->second].gotime = iter->go_time_PA;
        m_car[itera->second].road_path_id = iter->path_PA;
        m_car[itera->second].cross_path_index = iter->cross_path_PA;
        for(uint i=0; i<m_car[itera->second].road_path_id.size(); i++)//初始化m_car里road_path_index
        {
            auto iterb = road_map.find(m_car[itera->second].road_path_id[i]);
            m_car[itera->second].road_path_index.emplace_back(iterb->second);
        }
        pcar.emplace_back(m_car[itera->second]);
    }
    for(auto iter=pcar.begin();iter!=pcar.end();iter++)
    {
        for(auto iterp=iter->cross_path_index.begin();iterp!=iter->cross_path_index.end();iterp++)
        {
            auto iterfind=find(over_cross.begin(),over_cross.end(),*iterp);
            if(iterfind!=over_cross.end())
            {
                iter->pcar_overnum++;
            }
        }
    }
    sort(pcar.begin(),pcar.end(),m_pfun);
    vector<int> change_pcar;
    for(uint i=0;i<320;i++)
    {
        change_pcar.emplace_back(pcar[i].car_index);//m_car里的索引
    }

    vector<Car> already_change_car;
    int num_change_car = 0;
    for(auto iter = change_pcar.begin(); iter!=change_pcar.end();iter++)
    {
        //先判断路的起点和终点有没有被删除 没有被删除才能重新规划
        auto iter8 = find(over_cross.begin(), over_cross.end(), m_car[*iter].start_ind);
        auto iter9 = find(over_cross.begin(), over_cross.end(), m_car[*iter].end_ind);
        if(iter8 == over_cross.end() && iter9 == over_cross.end())
        {
            //判断出发点和终点点之间是联通的
            //cout << "重新规划的车辆" << endl;
           // cout << op_car[*iter].car_index << endl;

            if(again_ss_path_value[m_car[*iter].start_ind][m_car[*iter].end_ind] > 0 && again_ss_path_value[m_car[*iter].start_ind][m_car[*iter].end_ind] < 2000000)
            {
                //如果车的路径改变，同时对ss_path进行更新，便于后期排序
                //ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind] = again_ss_path[op_car[*iter].start_ind][op_car[*iter].end_ind];
                //对车的路径进行更新
                m_car[*iter].cross_path_index = again_ss_path[m_car[*iter].start_ind][m_car[*iter].end_ind];
                already_change_car.emplace_back(m_car[*iter]);
                num_change_car++;
            }
        }
    }
    V2Road_init(already_change_car,m_road,V2Rmat);
    cout << "已重新规划路线的车的数目为:" << num_change_car << endl;
    return already_change_car;
}

