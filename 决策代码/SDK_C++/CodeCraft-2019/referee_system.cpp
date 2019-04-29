#include "referee_system.h"
#include <set>
#include <math.h>
#include<iomanip>
#define COR1 0
#define COR2 1

#define END  0
#define WAIT 1

#define INCROSS  0
#define OUTCROSS 1
#define ALLDIR   2

vector<int> statistics(vector<Road>&m_road, vector<Car>&m_car)
{
    int wait_size=0;
    int stop_size=0;
    vector<int> statis_isRunning;
    for(uint i=0; i<m_road.size(); i++)
    {
        for(uint n=0; n<m_road[i].car_on_road_from.size(); n++)
        {
            for(uint m=0; m<m_road[i].car_on_road_from[n].size(); m++)
            {
                if(m_road[i].car_on_road_from[n][m] != -1)
                {
                    int car_index = m_road[i].car_on_road_from[n][m];
                    if(m_car[car_index].state == WAIT)
                    {
                        statis_isRunning.emplace_back(car_index);
                        wait_size++;
                    }else if(m_car[car_index].state == END)
                    {
                        stop_size++;
                    }else
                    {
                        cout<<"error4:不可能事件"<<endl;
                    }
                }
            }
        }
        for(uint n=0; n<m_road[i].car_on_road_to.size(); n++)
        {
            for(uint m=0; m<m_road[i].car_on_road_to[n].size(); m++)
            {
                if(m_road[i].car_on_road_to[n][m] != -1)
                {
                    int car_index = m_road[i].car_on_road_to[n][m];
                    if(m_car[car_index].state == WAIT)
                    {
                        statis_isRunning.emplace_back(car_index);
                        wait_size++;
                    }else if(m_car[car_index].state == END)
                    {
                        stop_size++;
                    }else
                    {
                        cout<<"error4:不可能事件"<<endl;
                    }
                }
            }
        }
    }
    return statis_isRunning;
}
//两个vector求交集
vector<int> vectors_intersection(vector<int> v1,vector<int> v2){
    vector<int> v;
    sort(v1.begin(),v1.end());
    sort(v2.begin(),v2.end());
    set_intersection(v1.begin(),v1.end(),v2.begin(),v2.end(),back_inserter(v));//求交集
    return v;
}
int divround(int a, int b)
{
    return round(a*100000.0/b);
}
void calculate_param(Referee &m_referee, vector<Car>&m_car)
{
    m_referee.car_size = m_car.size();
    int maxspeed = 0, minspeed = __INT_MAX__,primaxspeed = 0, priminspeed = __INT_MAX__;
    int lastgotime = 0, firstgotime = __INT_MAX__, prilastgotime = 0, prifirstgotime = __INT_MAX__;
    set<int> all_start_temp, all_end_temp, pri_start_temp, pri_end_temp;
    for(uint i=0;i<m_car.size();i++)
    {
        if(m_car[i].priority == 1)
        {
            m_referee.pri_car_size++;
            if(m_car[i].plantime < m_referee.first_pri_plantime) m_referee.first_pri_plantime = m_car[i].plantime;

            if(primaxspeed < m_car[i].speedmax) primaxspeed = m_car[i].speedmax;
            if(priminspeed > m_car[i].speedmax) priminspeed = m_car[i].speedmax;

            if(prilastgotime < m_car[i].plantime) prilastgotime = m_car[i].plantime;
            if(prifirstgotime > m_car[i].plantime) prifirstgotime = m_car[i].plantime;

            pri_start_temp.insert(m_car[i].start_ind);
            pri_end_temp.insert(m_car[i].end_ind);

        }else
        {
            if(lastgotime < m_car[i].plantime) lastgotime = m_car[i].plantime;
            if(firstgotime > m_car[i].plantime) firstgotime = m_car[i].plantime;

            if(maxspeed < m_car[i].speedmax) maxspeed = m_car[i].speedmax;
            if(minspeed > m_car[i].speedmax) minspeed = m_car[i].speedmax;
        }
        all_start_temp.insert(m_car[i].start_ind);
        all_end_temp.insert(m_car[i].end_ind);
    }

    if(primaxspeed > maxspeed) maxspeed = primaxspeed;
    if(priminspeed < minspeed) minspeed = priminspeed;
    m_referee.allcar_speedmax = maxspeed;
    m_referee.allcar_speedmin = minspeed;
    m_referee.pricar_speedmax = primaxspeed;
    m_referee.pricar_speedmin = priminspeed;

    if(prilastgotime > lastgotime) lastgotime = prilastgotime;
    if(prifirstgotime < firstgotime) firstgotime = prifirstgotime;
    m_referee.allcar_lastgotime = lastgotime;
    m_referee.allcar_firstgotime = firstgotime;
    m_referee.pricar_lastgotime = prilastgotime;
    m_referee.pricar_firstgotime = prifirstgotime;

    m_referee.allcar_startdistr = all_start_temp.size();
    m_referee.allcar_enddistr = all_end_temp.size();
    m_referee.pricar_startdistr = pri_start_temp.size();
    m_referee.pricar_enddistr = pri_end_temp.size();

    int para_1 = divround(m_referee.car_size,m_referee.pri_car_size);
    int para_2 = divround(divround(m_referee.allcar_speedmax,m_referee.allcar_speedmin),divround(m_referee.pricar_speedmax,m_referee.pricar_speedmin));
    int para_3 = divround(divround(m_referee.allcar_lastgotime,m_referee.allcar_firstgotime),divround(m_referee.pricar_lastgotime,m_referee.pricar_firstgotime));
    int para_4 = divround(m_referee.allcar_startdistr,m_referee.pricar_startdistr);
    int para_5 = divround(m_referee.allcar_enddistr,m_referee.pricar_enddistr);
    m_referee.para_a = para_1*0.05 + para_2*0.2375 + para_3*0.2375 + para_4*0.2375 + para_5*0.2375;
    m_referee.para_a = m_referee.para_a * 0.00001;
    m_referee.para_b = para_1*0.8 + para_2*0.05 + para_3*0.05 + para_4*0.05 + para_5*0.05;
    m_referee.para_b = m_referee.para_b * 0.00001;
}
bool referee_machine(vector<Cross> &m_cross, vector<Road>&m_road, vector<Car>&m_car)
{
    std::string ref_info("/home/chi/Ref_system/config/ref_info.txt");
    Referee m_referee;
    m_referee.cross_asinit(m_cross);
    calculate_param(m_referee, m_car);
    vector<int> Timenow_arr;
    vector<int> isRunning_arr;
    vector<int> Arriver_arr;
    while(true)
    {
        m_referee.stop_car_size = 0;
        m_referee.TimeNow++;
        driveJustCurrentRoad(m_road,m_car,m_referee);
        driveCarInitList(true,m_road,m_car,m_referee);
        createCarSequeue(m_road,m_car,m_cross);

        if(!driveCarInWaitState(m_cross,m_road,m_car,m_referee))//驱动所有等待车辆进入终止状态，死锁则退出
        {
            return false;
        }
        driveCarInitList(false,m_road,m_car,m_referee);
        if(m_referee.isFinish(m_car))//所有车辆到达目的地
        {
            cout<<"specialResult is Result{scheduleTime = "<<m_referee.Tpri<<", allScheduleTime = "<<m_referee.Tsum_pri<<"}"<<endl;
            cout<<"originResult is Result{scheduleTime = "<<m_referee.TimeNow<<", allScheduleTime = "<<m_referee.Tsum<<"}"<<endl;
            cout<<fixed<<setprecision(0)<<"CodeCraftJudge end schedule time is "<<round(m_referee.para_a*m_referee.Tpri + m_referee.TimeNow)<<", allScheduleTime = "<<round(m_referee.para_b*m_referee.Tsum_pri + m_referee.Tsum)<<"}"<<endl;
            break;
        }
        Timenow_arr.emplace_back(m_referee.TimeNow); isRunning_arr.emplace_back(m_referee.isRunning.size()); Arriver_arr.emplace_back(m_referee.Arrive.size());
        cout<<"Time_now: "<<m_referee.TimeNow<<"-------isRunning: "<<m_referee.isRunning.size()<<"-------Arrive: "<<m_referee.Arrive.size()<<endl;
    }
    print_vex(Timenow_arr,isRunning_arr,Arriver_arr, ref_info);
    return true;
}

bool Referee::isFinish(vector<Car> m_car)
{
    if(Arrive.size() == m_car.size())
        return true;
    else
        return false;
}
bool sortcross(const Cross &p1, const Cross &p2)
{
    return (p1.cross_id < p2.cross_id);
}

void Referee::cross_asinit(vector<Cross> m_cross)
{
    sort(m_cross.begin(),m_cross.end(),sortcross);
    for(auto iter=m_cross.begin();iter!=m_cross.end();iter++)
    {
        cross_ascend_index.emplace_back(iter->cross_index);
    }
}


void Init_all(vector<Cross> &m_cross, vector<Road> &m_road, vector<Car> &m_car, map<int, int> &cross_map, map<int, int> &road_map, map<int, int> &car_map)
{
    Cross_map(m_cross, cross_map);
    Road_map(m_road,cross_map,road_map);
    //映射
    for(uint i=0; i<m_cross.size(); i++)
    {
        for(uint j=0; j<m_cross[i].cross_road_sort.size(); j++)
        {
            if(m_cross[i].cross_road_sort[j] != -1)
            {
                auto iter = road_map.find(m_cross[i].cross_road_sort[j]);
                m_cross[i].cross_road_index.push_back(iter->second);
            }else {
                m_cross[i].cross_road_index.push_back(-1);
            }
        }
    }

}
void referee_init(vector<Road> &m_road,vector<Answer> &m_answer, vector<presetCar> &m_pcar,vector<Car> &m_car,vector<Car> &op_car, map<int, int> &car_map,map<int, int> &road_map)
{
    Loading_localanswer(m_road,m_answer,m_pcar,m_car,op_car,car_map,road_map);
    carport_init(m_road, m_car);
}
void Cross_map(vector<Cross> &m_cross, map<int, int> &cross_map)
{
    for(uint i=0; i<m_cross.size(); i++)
    {
        cross_map.insert(pair<int, int>(m_cross[i].cross_id, m_cross[i].cross_index));
    }
}

void Road_map(vector<Road> &m_road,map<int, int> &cross_map, map<int, int> &road_map)
{
    for(uint i=0; i<m_road.size(); i++)
    {
        road_map.insert(pair<int, int>(m_road[i].road_id, m_road[i].road_index));
        auto iter = cross_map.find(m_road[i].start_ind);
        m_road[i].start_ind = iter->second;
        auto iter1 = cross_map.find(m_road[i].end_ind);
        m_road[i].end_ind = iter1->second;
    }
}
void Car_map(vector<Car> &m_car, map<int, int> &cross_map, map<int, int> &car_map)
{
    for(uint i=0; i<m_car.size(); i++)
    {
        car_map.insert(pair<int, int>(m_car[i].car_id, m_car[i].car_index));
        auto iter = cross_map.find(m_car[i].start_ind);
        m_car[i].start_ind = iter->second;
        auto iter1 = cross_map.find(m_car[i].end_ind);
        m_car[i].end_ind = iter1->second;
    }
}
void Car_map_one(vector<Car> &m_car, map<int, int> &cross_map)
{
    for(uint i=0; i<m_car.size(); i++)
    {
        auto iter = cross_map.find(m_car[i].start_ind);
        m_car[i].start_ind = iter->second;
        auto iter1 = cross_map.find(m_car[i].end_ind);
        m_car[i].end_ind = iter1->second;
    }
}
void Car_map_two(vector<Car> &op_car, map<int, int> &car_map)
{
    car_map.clear();
    for(uint i=0; i<op_car.size(); i++)
    {
        op_car[i].car_index = i;
        car_map.insert(pair<int, int>(op_car[i].car_id, op_car[i].car_index));
    }
}
void Car_map_three(vector<Car> &opp_car)
{
    for(uint i=0; i<opp_car.size(); i++)
    {
        opp_car[i].op_car_index = opp_car[i].car_index;
        opp_car[i].car_index = i;
    }
}



/*
*函数名称：Loading_local
*函数功能：将答案中的车的出发时间赋值到m_car中，并初始化m_car的car_port
*函数参数：
*函数返回值：无
*Author:学学没完
*date：2019.4.7 17:06
*/
void Loading_localanswer(vector<Road> &m_road,vector<Answer> &m_answer,vector<presetCar> &m_pcar,vector<Car> &m_car,vector<Car> &op_car, map<int, int> &car_map,map<int, int> &road_map)//保证all_car的size和m_car一样大
{
    //道路answer信息
    for(uint i=0; i<m_answer.size();i++)//将answer中的信息赋值到all_car中
    {
        if(i == 46823)
        {
            cout<<"qwe"<<i<<endl;
        }
        m_car[m_answer[i].car_index].gotime = m_answer[i].gotime;
        m_car[m_answer[i].car_index].road_path_id = op_car[m_answer[i].op_car_index].road_path_id;
        m_car[m_answer[i].car_index].road_path_index = op_car[m_answer[i].op_car_index].road_path_index;
    }
    for(auto iter=m_pcar.begin();iter!=m_pcar.end();iter++)
    {
        auto itera = car_map.find(iter->car_id_PA);
        m_car[itera->second].gotime = iter->go_time_PA;
        m_car[itera->second].road_path_id = iter->path_PA;
        for(uint i=0; i<m_car[itera->second].road_path_id.size(); i++)//初始化m_car里road_path_index
        {
            auto iterb = road_map.find(m_car[itera->second].road_path_id[i]);
            m_car[itera->second].road_path_index.emplace_back(iterb->second);
        }
    }

    //补全车辆信息 初始化m_road里的carport;
    for(auto iter=m_car.begin();iter!=m_car.end();iter++)
    {
        if(iter->start_ind == m_road[iter->road_path_index[0]].start_ind)
        {
            m_road[iter->road_path_index[0]].carport_from.emplace_back(iter->car_index);//初始化m_road里的carport;
        }else {
            m_road[iter->road_path_index[0]].carport_to.emplace_back(iter->car_index);//初始化m_road里的carport;
        }
    }
}
//将车辆路径加载到车辆对象
void Loading(vector<Road> &m_road,vector<Answer> &m_answer, vector<presetCar> &m_pcar, vector<Car> &m_car, vector<Cross> &m_cross,map<int, int> &car_map,map<int, int> &road_map)//保证all_car的size和m_car一样大
{
    vector<Car> all_car = m_car;//只要得到和m_car大小一致的即可
    //道路answer信息
    for(uint i=0; i<m_answer.size();i++)//将answer中的信息赋值到all_car中
    {
        all_car[i].car_id = m_answer[i].car_id;
        all_car[i].gotime = m_answer[i].gotime;
        all_car[i].road_path_id =m_answer[i].car_path;
    }
    for(uint i=0; i<m_pcar.size();i++)//将panswer中的信息赋值到all_car中
    {
        all_car[m_answer.size()+i].car_id = m_pcar[i].car_id_PA;
        all_car[m_answer.size()+i].gotime = m_pcar[i].go_time_PA;
        all_car[m_answer.size()+i].road_path_id =m_pcar[i].path_PA;
    }
    //补全车辆信息
    for(auto iter=all_car.begin();iter!=all_car.end();iter++)
    {
        auto itera = car_map.find(iter->car_id);
        //iter->car_index = itera->second;
        m_car[itera->second].gotime = iter->gotime;
        m_car[itera->second].road_path_id = iter->road_path_id;
        for(uint i=0; i<m_car[itera->second].road_path_id.size(); i++)//初始化m_car里road_path_index
        {
            auto iterb = road_map.find(m_car[itera->second].road_path_id[i]);
            m_car[itera->second].road_path_index.emplace_back(iterb->second);
        }
        if(m_car[itera->second].start_ind == m_road[m_car[itera->second].road_path_index[0]].start_ind)
        {
            m_road[m_car[itera->second].road_path_index[0]].carport_from.emplace_back(itera->second);//初始化m_road里的carport;
        }else {
            m_road[m_car[itera->second].road_path_index[0]].carport_to.emplace_back(itera->second);//初始化m_road里的carport;
        }
//        if(m_car[itera->second].cross_path_index.empty())
//            Road2Cross(m_car[itera->second],m_road,road_map);//将m_car里的cross_path_index初始化
    }
    //映射
    for(uint i=0; i<m_cross.size(); i++)
    {
        for(uint j=0; j<m_cross[i].cross_road_sort.size(); j++)
        {
            if(m_cross[i].cross_road_sort[j] != -1)
            {
                auto iter = road_map.find(m_cross[i].cross_road_sort[j]);
                m_cross[i].cross_road_index.push_back(iter->second);
            }else {
                m_cross[i].cross_road_index.push_back(-1);
            }
        }
    }
}

//自定义排序函数
bool sortFun(const Car &p1, const Car &p2)
{
    if(p1.priority != p2.priority) return p1.priority > p2.priority;
    if(p1.gotime != p2.gotime) return p1.gotime < p2.gotime;
    return (p1.car_id < p2.car_id);
}

void carport_init(vector<Road> &m_road, vector<Car> &m_car)
{
    vector<Car> que_car;
    for(auto iter=m_road.begin();iter!=m_road.end();iter++)
    {
        que_car.clear();
        for(auto itera=iter->carport_from.begin(); itera!=iter->carport_from.end(); itera++)
        {
            que_car.emplace_back(m_car[*itera]);//每个路口的车都暂存在que_car中
        }
        iter->carport_from.clear();
        sort(que_car.begin(),que_car.end(),sortFun);//对该路口的所有车que_car按优先级>出发时间>车id排序
        for(auto iterb=que_car.begin(); iterb!=que_car.end(); iterb++)
        {
            iter->carport_from.emplace_back(iterb->car_index);//将该路口排好序的车重新放进车库中
        }

        que_car.clear();
        for(auto itera=iter->carport_to.begin(); itera!=iter->carport_to.end(); itera++)
        {
            que_car.emplace_back(m_car[*itera]);//每个路口的车都暂存在que_car中
        }
        iter->carport_to.clear();
        sort(que_car.begin(),que_car.end(),sortFun);//对该路口的所有车que_car按优先级>出发时间>车id排序
        for(auto iterb=que_car.begin(); iterb!=que_car.end(); iterb++)
        {
            iter->carport_to.emplace_back(iterb->car_index);//将该路口排好序的车重新放进车库中
        }
    }
}
/*
*函数名称：Road2Cross
*函数功能：获取s_car车的cross_path_index路径
*函数参数：
*函数返回值：无
*Author:学学没完
*date：2019.4.2 21:41
*/
void Road2Cross(Car &s_car, vector<Road>&m_road, map<int, int> &road_map)
{
    int road_index;
    int next_start_id = s_car.start_ind;
    for(uint i=0; i<s_car.road_path_id.size(); i++)
    {
        auto iter = road_map.find(s_car.road_path_id[i]);
        road_index = iter->second;
        if(m_road[road_index].start_ind == next_start_id)
        {
            s_car.cross_path_index.emplace_back(m_road[road_index].start_ind);
            next_start_id = m_road[road_index].end_ind;
        }else if(m_road[road_index].end_ind == next_start_id)
        {
            s_car.cross_path_index.emplace_back(m_road[road_index].start_ind);
            next_start_id = m_road[road_index].start_ind;
        }
    }
    s_car.cross_path_index.emplace_back(s_car.end_ind);
}

void driveJustCurrentRoad_part_pa(Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee, int channel)
{
    int car_rank_in_channle=0;//0代表是该通道的第一辆车
    vector<int> car_pos;//记录car_rank_in_channle对应的位置（m值）
    for(uint m=0; m<car_on_road[channel].size(); m++)//m=0车道第一排
    {
        if(car_on_road[channel][m]!=-1)
        {
            int car_index = car_on_road[channel][m];
            car_pos.emplace_back(m);
            m_car[car_index].speed_now = min(m_car[car_index].speedmax,s_road.speedlim);
            if(car_rank_in_channle == 0)//如果是第一排的车
            {
                if(car_pos[car_rank_in_channle] >= m_car[car_index].speed_now){//如果不能出路口
                    car_pos[car_rank_in_channle] = m - m_car[car_index].speed_now;
                    car_on_road[channel][m] = -1;
                    car_on_road[channel][m - m_car[car_index].speed_now] = car_index;
                    m_referee.stop_car_size++;
                    m_car[car_index].state = END;
                }else{
                    m_referee.wait_car_size++;
                    m_car[car_index].state = WAIT;
                }
            }else
            {
                int dis_car = car_pos[car_rank_in_channle] - car_pos[car_rank_in_channle-1] - 1;//该车与前车的距离
                if(dis_car < m_car[car_index].speed_now)//S1范围内有前车
                {
                    if(m_car[car_on_road[channel][car_pos[car_rank_in_channle-1]]].state == WAIT){//如果前车的状态是等待
                       m_car[car_index].state = WAIT;
                       m_referee.wait_car_size++;
                    }else{
                        m_car[car_index].speed_now = min(m_car[car_index].speed_now,dis_car);
                        car_pos[car_rank_in_channle] = m - m_car[car_index].speed_now;
                        car_on_road[channel][m] = -1;
                        car_on_road[channel][m - m_car[car_index].speed_now] = car_index;
                        m_referee.stop_car_size++;
                        m_car[car_index].state = END;
                    }
                }else{//无前车
                    car_pos[car_rank_in_channle] = m - m_car[car_index].speed_now;
                    car_on_road[channel][m] = -1;
                    car_on_road[channel][m - m_car[car_index].speed_now] = car_index;
                    m_referee.stop_car_size++;
                    m_car[car_index].state = END;
                }
            }
            car_rank_in_channle++;
        }
    }
}
void driveJustCurrentRoad_part(Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee)
{
    for(uint n=0; n<car_on_road.size(); n++)//n=0车道最高优先级通道
    {
        driveJustCurrentRoad_part_pa(s_road,m_car,car_on_road,m_referee,n);
    }
}

void driveJustCurrentRoad(vector<Road>&m_road, vector<Car>&m_car, Referee &m_referee)
{
    for(uint road_index=0; road_index<m_road.size(); road_index++)
    {
        driveJustCurrentRoad_part(m_road[road_index],m_car,m_road[road_index].car_on_road_from,m_referee);
        driveJustCurrentRoad_part(m_road[road_index],m_car,m_road[road_index].car_on_road_to,m_referee);
    }
}


//只能用在车库的车上下一条路的
bool runToRoad(Car&s_car,Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee)
{
    s_car.speed_now = min(s_road.speedlim, s_car.speedmax);
    for(uint n=0; n<car_on_road.size(); n++)
    {
        bool channel_have_car = false;//判断该通道是否有车的标志位 0就是没车 1就是有车
        for(uint m=s_road.length-1; m>=s_road.length-s_car.speed_now; m--)
        {
            int front_car_index = car_on_road[n][m];
            if(front_car_index != -1)
            {
                channel_have_car = true;
                if(m_car[front_car_index].state == END)//当前车的状态为终止状态
                {
                    if(m == s_road.length-1)//如果前车在最后一个位置，即该通道没位置
                    {
                        if(n == car_on_road.size()-1)//如果是所有通道全部堵死
                        {
                            return false;
                        }
                        break;//跳出当前通道，搜索下一通道
                    }else {
                        car_on_road[n][m+1] = s_car.car_index;
                        s_car.state = END;
                        m_referee.stop_car_size++;
                        return true;
                    }
                }else {//当前车的状态为等待状态
                    return false;
                }
            }
        }
        if(!channel_have_car)//当前通道在该车S2内没车，直接上车
        {
            car_on_road[n][s_road.length-s_car.speed_now] = s_car.car_index;
            m_referee.stop_car_size++;
            s_car.state = END;
            return true;
        }
    }
}

void runCarInInitList_part(Road &s_road, vector<Car>&m_car, vector<int> &caport, vector<vector<int>> &car_on_road, Referee &m_referee, bool priority)
{
    for(auto iter=caport.begin();iter!=caport.end();)
    {
        int car_index = *iter;
        if(m_referee.TimeNow < m_car[car_index].gotime)//如果当前时间还没到该车的出发时间则不发该车
        {
            iter++;
            continue;
        }
        if(priority)
        {
            if(m_car[car_index].priority == 0)
                break;
        }
        if(runToRoad(m_car[car_index],s_road,m_car,car_on_road,m_referee)){
            iter = caport.erase(iter);
            m_referee.isRunning.emplace_back(car_index);//将车辆的索引放到isRunning中
        }else {
            iter++;
        }
    }
}

void runCarInInitList(Road &s_road, vector<Car>&m_car, Referee &m_referee, bool priority)
{
    runCarInInitList_part(s_road,m_car,s_road.carport_from,s_road.car_on_road_from,m_referee,priority);
    runCarInInitList_part(s_road,m_car,s_road.carport_to,s_road.car_on_road_to,m_referee,priority);
}

//priority 为true时只上优先车辆，false上全部车辆
void driveCarInitList(bool priority, vector<Road>&m_road, vector<Car>&m_car, Referee &m_referee)
{
    for(uint road_index=0; road_index<m_road.size(); road_index++)
    {
        runCarInInitList(m_road[road_index],m_car,m_referee,priority);
    }
}

bool sortcarque(const Car &p1, const Car &p2)
{
    if(p1.priority != p2.priority) return p1.priority > p2.priority;
    if(p1.car_pos != p2.car_pos) return p1.car_pos < p2.car_pos;
    return (p1.channle < p2.channle);
}
void createCarSequeue_part(vector<Road>&m_road, vector<Car>&m_car, int road_index, int cross_index)
{
    if(cross_index == m_road[road_index].end_ind)
    {
        m_road[road_index].pri_queue_from.clear();
        //该路的一个方向
        for(uint n=0; n<m_road[road_index].car_on_road_from.size();n++)
        {
            for(uint m=0; m<m_road[road_index].car_on_road_from[n].size(); m++)
            {
                if(m_road[road_index].car_on_road_from[n][m] != -1)
                {
                    int car_index = m_road[road_index].car_on_road_from[n][m];
                    if(m_car[car_index].state == WAIT)
                    {
                        m_road[road_index].pri_queue_from.emplace_back(car_index);
                        m_car[car_index].car_pos = m;
                        m_car[car_index].channle = n;
                    }
                    break;//只找每个通道第一辆车，无论是什么状态
                }
            }
        }
        vector<Car> car_que;
        for(uint i=0; i<m_road[road_index].pri_queue_from.size();i++)
        {
            car_que.emplace_back(m_car[m_road[road_index].pri_queue_from[i]]);
        }
        m_road[road_index].pri_queue_from.clear();
        sort(car_que.begin(),car_que.end(),sortcarque);
        for(uint i=0; i<car_que.size();i++)
        {
            m_road[road_index].pri_queue_from.emplace_back(car_que[i].car_index);
        }
    }else
    {
        m_road[road_index].pri_queue_to.clear();
        //该路的另一个方向
        for(uint n=0; n<m_road[road_index].car_on_road_to.size();n++)
        {
            for(uint m=0; m<m_road[road_index].car_on_road_to[n].size(); m++)
            {
                if(m_road[road_index].car_on_road_to[n][m] != -1)
                {
                    int car_index = m_road[road_index].car_on_road_to[n][m];
                    if(m_car[car_index].state == WAIT)
                    {
                        m_road[road_index].pri_queue_to.emplace_back(car_index);
                        m_car[car_index].car_pos = m;
                        m_car[car_index].channle = n;
                    }
                    break;//只找每个通道第一辆车，无论是什么状态
                }
            }
        }
        vector<Car> car_que;
        for(uint i=0; i<m_road[road_index].pri_queue_to.size();i++)
        {
            car_que.emplace_back(m_car[m_road[road_index].pri_queue_to[i]]);
        }
        m_road[road_index].pri_queue_to.clear();
        sort(car_que.begin(),car_que.end(),sortcarque);
        for(uint i=0; i<car_que.size();i++)
        {
            m_road[road_index].pri_queue_to.emplace_back(car_que[i].car_index);
        }
    }
}
void createCarSequeue(vector<Road>&m_road, vector<Car>&m_car, vector<Cross>&m_cross)
{
    for(uint cross_index=0; cross_index < m_cross.size(); cross_index++)
    {
        for(uint k=0; k<m_cross[cross_index].cross_road_index.size(); k++)
        {
            int road_index = m_cross[cross_index].cross_road_index[k];
            if(road_index != -1)
            {
                createCarSequeue_part(m_road,m_car,road_index,cross_index);
            }
        }
    }
}

int GetS2(Car &s_car, Road &next_road, int S1)
{
    int S2 = min(s_car.speedmax,next_road.speedlim) -S1;
    if(S2<0) S2 = 0;
    return S2;
}
int Get_next_road_index(Car &s_car, int road_index_now)
{
    int next_road_index = -1;
    for(uint i=0; i<s_car.road_path_index.size(); i++)
    {
        if(s_car.road_path_index[i] == road_index_now){
            if(i != s_car.road_path_index.size()-1){
                next_road_index = s_car.road_path_index[i+1];
            }else{
                next_road_index = -1;//到最后一条路了，
            }
            break;
        }
    }
    return next_road_index;
}

//直行0,左转1,右转2
int Get_Dir(int now_road, int next_road)
{
    int dir_state = -1;
    int road_flag = next_road - now_road;
    if(abs(road_flag)==2)
    {
        return dir_state = 0;
    }else if(road_flag==1||road_flag==-3)
    {
        return dir_state = 1;
    }else {
        return dir_state = 2;
    }
}

int fake_next_road(Road&s_road,Cross&s_cross)
{
    int next_road = -1;
    int road_rank = -1;
    for(uint k=0;k<s_cross.cross_road_index.size(); k++)
    {
        if(s_road.road_index == s_cross.cross_road_index[k])
        {
            road_rank = s_cross.cross_road_rank[k];
        }
    }
    int next_rank=0;
    if(road_rank == 0||road_rank == 1)
    {
        next_rank = road_rank + 2;
    }else {
        next_rank = road_rank - 2;
    }
    for(uint k=0;k<s_cross.cross_road_rank.size(); k++)
    {
        if(next_rank == s_cross.cross_road_rank[k])
        {
            next_road = s_cross.cross_road_index[k];
        }
    }
    return next_road;
}
bool sortconflict(const Car &p1, const Car &p2)
{
    if(p1.priority != p2.priority) return p1.priority > p2.priority;
    if(p1.car_dir != p2.car_dir) return p1.car_dir < p2.car_dir;
}

bool conflict(vector<Cross> m_cross, vector<Road>&m_road, vector<Car>&m_car, int cross_index_now, int road_index_now,int next_road_index, int car_index)
{
    vector<int> seque_conflict;
    if(next_road_index==-1)
    {
        next_road_index = fake_next_road(m_road[road_index_now], m_cross[cross_index_now]);
        if(next_road_index==-1)
            return false;
    }
    for(uint i=0; i<m_cross[cross_index_now].cross_road_index.size(); i++)
    {
        int road_index = m_cross[cross_index_now].cross_road_index[i];
        if(road_index != -1)
        {
            if(cross_index_now == m_road[road_index].end_ind)
            {
                //car_on_road_from
                if(!m_road[road_index].pri_queue_from.empty())
                {
                    int other_car_index = m_road[road_index].pri_queue_from[0];
                    int other_car_next_road_index = Get_next_road_index(m_car[other_car_index],road_index);
                    if(other_car_next_road_index==-1)
                        other_car_next_road_index = fake_next_road(m_road[road_index], m_cross[cross_index_now]);
                    if(other_car_next_road_index == next_road_index)
                    {
                        seque_conflict.emplace_back(other_car_index);
                        if(other_car_next_road_index != -1)
                        {
                            int road_rank = m_cross[cross_index_now].cross_road_rank[i];
                            int next_road_rank = -1;
                            for(uint k=0;k<m_cross[cross_index_now].cross_road_index.size(); k++)
                            {
                                if(other_car_next_road_index == m_cross[cross_index_now].cross_road_index[k])
                                {
                                    next_road_rank = m_cross[cross_index_now].cross_road_rank[k];
                                }
                            }
                            if(next_road_rank == -1){cout<<"error2:不可能事件"<<endl;}
                            m_car[other_car_index].car_dir = Get_Dir(road_rank,next_road_rank);
                        }else{
                            m_car[other_car_index].car_dir = 0;
                        }
                    }
                }
            }else
            {
                if(m_road[road_index].isDuplex == 1)
                {
                    //car_on_road_to
                    if(!m_road[road_index].pri_queue_to.empty())
                    {
                        int other_car_index = m_road[road_index].pri_queue_to[0];
                        int other_car_next_road_index = Get_next_road_index(m_car[other_car_index],road_index);
                        if(other_car_next_road_index==-1)
                            other_car_next_road_index = fake_next_road(m_road[road_index], m_cross[cross_index_now]);
                        if(other_car_next_road_index == next_road_index)
                        {
                            seque_conflict.emplace_back(other_car_index);
                            if(other_car_next_road_index != -1)
                            {
                                int road_rank = m_cross[cross_index_now].cross_road_rank[i];
                                int next_road_rank = -1;
                                for(uint k=0;k<m_cross[cross_index_now].cross_road_index.size(); k++)
                                {
                                    if(other_car_next_road_index == m_cross[cross_index_now].cross_road_index[k])
                                    {
                                        next_road_rank = m_cross[cross_index_now].cross_road_rank[k];
                                    }
                                }
                                if(next_road_rank == -1){cout<<"error2:不可能事件"<<endl;}
                                m_car[other_car_index].car_dir = Get_Dir(road_rank,next_road_rank);
                            }else{
                                m_car[other_car_index].car_dir = 0;
                            }
                        }
                    }
                }
            }
        }
    }
    vector<Car> que_car;
    for(uint i=0; i<seque_conflict.size(); i++)
    {
        que_car.emplace_back(m_car[seque_conflict[i]]);
    }
    sort(que_car.begin(),que_car.end(),sortconflict);
    if(que_car[0].car_index == car_index)
        return false;
    else
        return true;
}

void single_driveCarInitList(bool priority, Road &s_road, vector<Car>&m_car,vector<vector<int>> &car_on_road,vector<int> &carport, Referee &m_referee)
{
    runCarInInitList_part(s_road,m_car,carport,car_on_road,m_referee,priority);
}

void driveCarInWaitState_part(vector<Road>&m_road, vector<Car>&m_car, vector<Cross> &m_cross, Referee &m_referee, vector<vector<int>> &car_on_road,vector<int> &carport, int cross_index_now, int road_index_now)
{
    if(cross_index_now == m_road[road_index_now].end_ind)
    {
        //car_on_road_from
        while(!m_road[road_index_now].pri_queue_from.empty()){
            int car_index = m_road[road_index_now].pri_queue_from[0];
            //pri_queue.erase(pri_queue.begin());
            int next_road_index = Get_next_road_index(m_car[car_index],road_index_now);
            if(conflict(m_cross,m_road,m_car,cross_index_now,road_index_now,next_road_index,car_index)) break;
            int S2 = -1;
            if(next_road_index != -1)
            {
                S2 = GetS2(m_car[car_index],m_road[next_road_index],m_car[car_index].car_pos);
            }
            if(movToNextRoad(next_road_index,m_car,car_index,m_road,cross_index_now,road_index_now,S2,m_car[car_index].channle,m_car[car_index].car_pos,car_on_road,m_referee)){
                driveJustCurrentRoad_single_channel(m_road[road_index_now],m_car,car_on_road,m_referee,m_car[car_index].channle);
                createCarSequeue_part(m_road,m_car,road_index_now,cross_index_now);
                single_driveCarInitList(true,m_road[road_index_now],m_car,car_on_road,carport,m_referee);
            }else {
                break;
            }
        }
    }else
    {
        if(m_road[road_index_now].isDuplex == 1)
        {
            //car_on_road_to
            while(!m_road[road_index_now].pri_queue_to.empty()){
                int car_index = m_road[road_index_now].pri_queue_to[0];
                //pri_queue.erase(pri_queue.begin());
                int next_road_index = Get_next_road_index(m_car[car_index],road_index_now);
                if(conflict(m_cross,m_road,m_car,cross_index_now,road_index_now,next_road_index,car_index)) break;
                int S2 = -1;
                if(next_road_index != -1)
                {
                    S2 = GetS2(m_car[car_index],m_road[next_road_index],m_car[car_index].car_pos);
                }
                if(movToNextRoad(next_road_index,m_car,car_index,m_road,cross_index_now,road_index_now,S2,m_car[car_index].channle,m_car[car_index].car_pos,car_on_road,m_referee)){
                    driveJustCurrentRoad_single_channel(m_road[road_index_now],m_car,car_on_road,m_referee,m_car[car_index].channle);
                    createCarSequeue_part(m_road,m_car,road_index_now,cross_index_now);
                    single_driveCarInitList(true,m_road[road_index_now],m_car,car_on_road,carport,m_referee);
                }else {
                    break;
                }
            }
        }
    }

}


bool driveCarInWaitState(vector<Cross> &m_cross, vector<Road>&m_road, vector<Car> &m_car, Referee &m_referee)
{
    while(m_referee.wait_car_size != 0)
    {
        int last_wait_car_size = m_referee.wait_car_size;
        for(uint i=0; i<m_referee.cross_ascend_index.size(); i++)
        {
            int cross_index = m_referee.cross_ascend_index[i];
            for(uint j=0; j<m_cross[cross_index].cross_road_index.size(); j++)
            {
                int road_index = m_cross[cross_index].cross_road_index[j];
                if(road_index != -1)//去除无效路段
                {
                    if(cross_index == m_road[road_index].end_ind)
                        driveCarInWaitState_part(m_road,m_car,m_cross,m_referee,m_road[road_index].car_on_road_from,m_road[road_index].carport_from,cross_index,road_index);
                    else
                        driveCarInWaitState_part(m_road,m_car,m_cross,m_referee,m_road[road_index].car_on_road_to,m_road[road_index].carport_to,cross_index,road_index);
                }
            }
        }
        if(m_referee.wait_car_size > 0 && m_referee.wait_car_size == last_wait_car_size)//判断出死锁
        {
            m_referee.death = 1;
            return false;
        }
    }
    return true;
}

void driveJustCurrentRoad_single_channel(Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee, int channel)
{
    int car_rank_in_channle=0;//0代表是该通道的第一辆车
    vector<int> car_pos;//记录car_rank_in_channle对应的位置（m值）
    for(uint m=0; m<car_on_road[channel].size(); m++)//m=0车道第一排
    {
        if(car_on_road[channel][m]!=-1)
        {
            int car_index = car_on_road[channel][m];
            car_pos.emplace_back(m);
            if(m_car[car_index].state == WAIT)
            {
                m_car[car_index].speed_now = min(m_car[car_index].speedmax,s_road.speedlim);
                if(car_rank_in_channle == 0)//如果是第一排的车
                {
                    if(car_pos[car_rank_in_channle] >= m_car[car_index].speed_now){//如果不能出路口
                        car_pos[car_rank_in_channle] = m - m_car[car_index].speed_now;
                        car_on_road[channel][m] = -1;
                        car_on_road[channel][m - m_car[car_index].speed_now] = car_index;
                        m_referee.stop_car_size++;
                        m_referee.wait_car_size--;
                        m_car[car_index].state = END;
                    }else{
                        break;
                    }
                }else
                {
                    int dis_car = car_pos[car_rank_in_channle] - car_pos[car_rank_in_channle-1] - 1;//该车与前车的距离
                    if(dis_car < m_car[car_index].speed_now)//S1范围内有前车
                    {
                        if(m_car[car_on_road[channel][car_pos[car_rank_in_channle-1]]].state == WAIT){//如果前车的状态是等待
                           cout<<"error3:不可能"<<endl;
                        }else{
                            m_car[car_index].speed_now = min(m_car[car_index].speed_now,dis_car);
                            car_pos[car_rank_in_channle] = m - m_car[car_index].speed_now;
                            car_on_road[channel][m] = -1;
                            car_on_road[channel][m - m_car[car_index].speed_now] = car_index;
                            m_referee.stop_car_size++;
                            m_referee.wait_car_size--;
                            m_car[car_index].state = END;
                        }
                    }else{//无前车
                        car_pos[car_rank_in_channle] = m - m_car[car_index].speed_now;
                        car_on_road[channel][m] = -1;
                        car_on_road[channel][m - m_car[car_index].speed_now] = car_index;
                        m_referee.stop_car_size++;
                        m_referee.wait_car_size--;
                        m_car[car_index].state = END;
                    }
                }
            }
            car_rank_in_channle++;
        }
    }
}
bool movToNextRoad_part(vector<Car> &m_car,vector<vector<int>> &next_car_on_road, vector<vector<int>> &car_on_road, int S2,int car_index, int car_x, int car_y, Referee &m_referee)
{
   if(S2 != 0)
   {
       for(uint n=0; n<next_car_on_road.size(); n++)
       {
           bool S2_have_car = false;
           for(uint m=next_car_on_road[n].size()-1; m>=next_car_on_road[n].size()-S2;m--)
           {
               if(next_car_on_road[n][m] != -1)//如果S2内有车
               {
                   int ncar_index = next_car_on_road[n][m];
                   S2_have_car = true;
                   if(m_car[ncar_index].state == WAIT)//如果前车是等待
                   {
                       m_car[car_index].state =WAIT;
                       return false;
                   }else {//如果前车是终止
                       if(m == next_car_on_road[n].size()-1){
                           if(n == next_car_on_road.size()-1)
                           {//全部堵死
                               m_referee.wait_car_size--;
                               m_referee.stop_car_size++;
                               car_on_road[car_x][car_y] = -1;
                               car_on_road[car_x][0] = car_index;
                               m_car[car_index].state = END;
                               return true;
                           }
                           break;
                       }else{//有位置上
                           m_referee.wait_car_size--;
                           m_referee.stop_car_size++;
                           car_on_road[car_x][car_y] = -1;
                           next_car_on_road[n][m+1] = car_index;
                           m_car[car_index].state = END;
                           return true;
                       }
                   }
               }
           }
           if(!S2_have_car)//当前车道没有车
           {
               m_referee.wait_car_size--;
               m_referee.stop_car_size++;
               car_on_road[car_x][car_y] = -1;
               next_car_on_road[n][next_car_on_road[n].size()-S2] = car_index;
               m_car[car_index].state = END;
               return true;
           }
       }
   }else
   {
       m_referee.wait_car_size--;
       m_referee.stop_car_size++;
       car_on_road[car_x][car_y] = -1;
       car_on_road[car_x][0] = car_index;
       m_car[car_index].state = END;
       return true;
   }

}

bool movToNextRoad(int next_road_index,vector<Car> &m_car,int car_index, vector<Road> &m_road, int cross_index_now, int road_index_now, int S2, int car_x,int car_y, vector<vector<int>> &car_on_road, Referee &m_referee)
{
    if(next_road_index != -1)
    {
        if(cross_index_now == m_road[next_road_index].start_ind)
        {
            //car_on_road_from
            if(movToNextRoad_part(m_car,m_road[next_road_index].car_on_road_from,car_on_road,S2,car_index,car_x,car_y,m_referee))
                return true;
            else
                return false;

        }else
        {
            if(m_road[next_road_index].isDuplex == 1)
            {
                //car_on_road_to
                if(movToNextRoad_part(m_car,m_road[next_road_index].car_on_road_to,car_on_road,S2,car_index,car_x,car_y,m_referee))
                    return true;
                else
                    return false;
            }
        }

    }else {//到最后一条路
        m_referee.wait_car_size--;
        car_on_road[car_x][car_y] = -1;
        auto iter = find(m_referee.isRunning.begin(),m_referee.isRunning.end(),car_index);
        m_referee.isRunning.erase(iter);
        m_referee.Arrive.emplace_back(car_index);
        m_referee.Tsum += (m_referee.TimeNow - m_car[car_index].plantime);
        if(m_car[car_index].priority == 1)
        {
            m_referee.Tpri = m_referee.TimeNow - m_referee.first_pri_plantime;
            m_referee.Tsum_pri += (m_referee.TimeNow - m_car[car_index].plantime);
        }

        return true;
    }

}



void print_vex(vector<int> Timenow_arr, vector<int> isRunning_arr, vector<int>Arriver_arr, string answerPath)
{
    //初始化输出txt
    string filepath = answerPath;
    ofstream put;
    put.open(filepath.c_str(), ios::app);

    put<<"Timenow     isRunning     Arriver"<<endl;
    for(uint i=0; i<Timenow_arr.size(); i++)
    {
        put<<Timenow_arr[i]<<","<<isRunning_arr[i]<<","<<Arriver_arr[i]<<endl;
    }
}



















