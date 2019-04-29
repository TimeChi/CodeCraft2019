#ifndef ZZCODE_H
#define ZZCODE_H
#include <set>
#include <sstream>
#include "data_pre_processing.h"

void pref_car_way_frirst(vector<Cross> &m_cross, vector<Road> &m_road, vector<Car> &op_car,vector<vector<int>> &ss_path_value, vector<vector<vector<int>>> &ss_path);

void pre_car_way_second(vector<Car> &m_car,vector<Cross> &m_cross, vector<Road> &m_road, vector<vector<vector<int>>> &ss_path);

//void pre_car_way_three(vector<Car> &m_car, vector<Road> m_road, vector<Cross> m_cross,vector<vector<vector<int>>> &ss_path);

//void write_car_way(string write_name, vector<Car> write_car);

//void write_road(string write_road_name, vector<Road> write_road);

//void write_cross(string write_cross_name, vector<Cross> write_cross);

bool sortover(Car&p1, Car&p2);

int SingleCar_CostTime(Car s_car, vector<Road> m_road);

vector<vector<int>> Vex2Roadid(const vector<Road> &m_road);

void s_carV2Road(Car &s_car,vector<Road> &m_road, vector<vector<int>> V2Rmat);

void V2Road_init(vector<Car> &m_car,vector<Road> &m_road, vector<vector<int>> V2Rmat);

void print_answertxt(vector<vector<int>> V2Rmat,int car_batch,int go_time,const vector<Car> &m_car,const vector<Road> m_road,string answerPath);

void Arrange2(vector<int> &gotime,  vector<presetCar> m_pcar, vector<int> &car_arrange_index,vector<Car> &op_car,vector<Road> m_road,vector<Cross> m_cross,string answerPath, int caronroad_threshold, int time_interval);

void Arrange_sur(vector<vector<vector<int>>> ss_path_p,vector<int> dense_iterval,vector<vector<int>> V2Rmat,vector<int> over_cross, vector<vector<vector<int>>> again_ss_path, vector<vector<int>> again_ss_path_value, vector<int> &gotime, vector<int> &car_arrange_index,vector<Car> &op_car,vector<Road> m_road,vector<Cross> m_cross,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g);

void Arrange_sur_part(vector<vector<vector<int>>> ss_path_p,vector<int> dense_iterval,vector<vector<int>> V2Rmat,vector<int> over_cross, vector<vector<vector<int>>> again_ss_path, vector<vector<int>> again_ss_path_value, vector<int> &gotime,vector<int> &car_arrange_index, vector<Car> &op_car,vector<int> car_wait,vector<Road> m_road,string answerPath, int caronroad_threshold, int time_interval,int para_c,int para_d,int para_e,int para_f,int para_g);

vector<int> pri_car_deal(vector<Car> &m_car,vector<Cross> &m_cross, vector<Road> &m_road,vector<presetCar> &m_pcar, signed int N, vector<vector<vector<int>>> &ss_path);
vector<int> pri_car_deal_part(vector<Cross> m_cross,vector<Car> &sur_car);

void Arrange_pri(vector<Car> &opp_car,vector<int> &gotime,vector<int> &car_arrange_index,vector<Car> &op_car,vector<Road> m_road,vector<int> car_wait_arrange_pri,string answerPath, int time_interval);

vector<int> pri_car_GetOverCross(vector<Cross> &m_cross, vector<presetCar> &m_pcar);//获取预置车辆过载路口

void Gen_new_pri_path(vector<Cross> &m_cross,vector<Road> &m_road,vector<int> over_cross,vector<vector<vector<int>>> &again_ss_path, vector<vector<int>> &again_ss_path_value);

void Change_singleCar_path(vector<int> over_cross ,Car &s_car,vector<vector<vector<int>>> again_ss_path, vector<vector<int>> again_ss_path_value);
void Change_singleCar_path(Car &s_car, vector<vector<vector<int>>> again_ss_path);

vector<int> Get_dense_iterval(vector<presetCar> &m_pcar);

#endif // ZZCODE_H
