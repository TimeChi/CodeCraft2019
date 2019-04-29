#include "presetanswer.h"

/*
*函数名称：PARoad2Cross
*函数功能：ps_car车的road_id的路径转化为cross_id路径
*函数参数：车辆数据 m_car
*函数返回值：无
*Author:学学没完
*date：2019.4.2 21:41
*/
void PARoad2Cross(Car &s_car, vector<Road>&m_road, presetCar &ps_car, map<int, int> &road_map)
{
    int road_index;
    int next_start_id = s_car.start_ind;
    for(uint i=0; i<ps_car.path_PA.size(); i++)
    {
        auto iter = road_map.find(ps_car.path_PA[i]);
        road_index = iter->second;
        if(m_road[road_index].start_ind == next_start_id)
        {
            ps_car.cross_path_PA.emplace_back(m_road[road_index].start_ind);
            next_start_id = m_road[road_index].end_ind;
        }else if(m_road[road_index].end_ind == next_start_id)
        {
            ps_car.cross_path_PA.emplace_back(m_road[road_index].end_ind);
            next_start_id = m_road[road_index].start_ind;
        }
    }
    ps_car.cross_path_PA.emplace_back(s_car.end_ind);
}


