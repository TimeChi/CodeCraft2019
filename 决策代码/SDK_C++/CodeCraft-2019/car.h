#ifndef CAR_H
#define CAR_H
#include <vector>



class Car
{
   public:
    //#(id,     from,     to,     speed,    planTime, priority, preset)
    int car_id, start_ind, end_ind, speedmax, plantime, priority, preset;

    //car now info
    int car_index;//car在m_car中的索引
    int gotime;//安排到出发时间
    int speed_now;  //该车当前速度
    int state; //车的状态
    std::vector<int> road_path_id;//记录的车的路径(路的id)
    std::vector<int> road_path_index;//记录的车的路径(路的index)
    std::vector<int> cross_path_index;//记录的车的路径（路口的index）

    int car_pos;//车在道路的位置
    int channle;//车所在道路的第几通道
    int car_dir;//车的转向

    int pcar_overnum=0;
    int op_car_index;
    int cost_time = 0;//单车无阻碍，花费时间
    int car_over_cross_cout = 0;
    Car(int cid, int sid,int eid,int smax,int gt,int pro,int pre,int ii);
    ~Car();
};

#endif // CAR_H
