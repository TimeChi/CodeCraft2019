#ifndef M_ALGORITHM_H
#define M_ALGORITHM_H
#include"zzcode.h"

void Init_Cost_Time(vector<Car> &op_car,vector<Road> m_road);

vector<Car> Get_deal_car(vector<Car> op_car);

void pre_car_way_second_v2(vector<Car> &op_car,vector<Car> &m_car,vector<Cross> &m_cross, vector<Road> &m_road, vector<vector<vector<int>>> &ss_path);

#endif // ALGORITHM_H
