#ifndef PRESETANSWER_H
#define PRESETANSWER_H
#include <vector>

class presetCar
{
public:
    int car_id_PA;
    int go_time_PA;
    std::vector<int> path_PA;
    presetCar(int car_id, int go_time)
    {
        car_id_PA = car_id;
        go_time_PA = go_time;
    }
};
#endif // PRESETANSWER_H
