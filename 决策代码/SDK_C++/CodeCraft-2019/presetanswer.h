#ifndef PRESETANSWER_H
#define PRESETANSWER_H
#include <vector>
#include "car.h"
#include "road.h"
#include "cross.h"
#include <map>
using namespace std;

class presetCar
{
public:
    int car_id_PA;
    int car_index_in_m_road;//在m_car中的索引，暂时没用，所以没有赋值
    int car_index;//在m_pcar中的索引
    int go_time_PA;
    std::vector<int> path_PA;
    std::vector<int> cross_path_PA;
    presetCar(int car_id, int go_time)
    {
        car_id_PA = car_id;
        go_time_PA = go_time;
    }
};

void PARoad2Cross(Car &s_car, vector<Road>&m_road, presetCar &ps_car,map<int, int> &road_map);
#endif // PRESETANSWER_H
