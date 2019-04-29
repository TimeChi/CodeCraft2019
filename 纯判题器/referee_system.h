#ifndef REFEREE_SYSTEM_H
#define REFEREE_SYSTEM_H

#include "data_pre_processing.h"
class Referee
{
public:
    vector<int> isRunning;
    vector<int> Arrive;
    int TimeNow = 0;
    int death = 0;
    int wait_car_size = 0;
    int stop_car_size = 0;
    int first_pri_plantime = __INT_MAX__;
    int Tpri = 0;
    int Tsum = 0;
    int Tsum_pri=0;

    double para_a;
    double para_b;
    int car_size, pri_car_size=0, allcar_speedmax, allcar_speedmin, pricar_speedmax, pricar_speedmin;
    int allcar_lastgotime,allcar_firstgotime,pricar_lastgotime,pricar_firstgotime;
    int allcar_startdistr,allcar_enddistr,pricar_startdistr,pricar_enddistr;
    vector<int> cross_ascend_index;
    Referee() {}
    bool isFinish(vector<Car> m_car);
    void cross_asinit(vector<Cross> m_cross);

};
bool referee_machine(vector<Cross> &m_cross, vector<Road>&m_road, vector<Car>&m_car);

void createCarSequeue(vector<Road>&m_road, vector<Car>&m_car, vector<Cross>&m_cross);

void Road2Cross(Car &s_car, vector<Road>&m_road, map<int, int> &road_map);

void Cross_map(vector<Cross> &m_cross, map<int, int> &cross_map);

void Road_map(vector<Road> &m_road,map<int, int> &cross_map, map<int, int> &road_map);

void Car_map(vector<Car> &m_car, map<int, int> &cross_map, map<int, int> &car_map);

void Loading(vector<Road> &m_road,vector<Answer> &m_answer, vector<presetCar> &m_pcar, vector<Car> &m_car, vector<Cross> &m_cross,map<int, int> &car_map,map<int, int> &road_map);

void Init_all(vector<Answer> &m_answer,vector<presetCar> &m_pcar,vector<Cross> &m_cross, vector<Road> &m_road, vector<Car> &m_car, map<int, int> &cross_map, map<int, int> &road_map, map<int, int> &car_map);

void carport_init(vector<Road> &m_road, vector<Car> &m_car);

bool runToRoad(Car&s_car,Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee);

void runCarInInitList_part(Road &s_road, vector<Car>&m_car, vector<int> &caport, vector<vector<int>> &car_on_road, Referee &m_referee, bool priority);
void runCarInInitList(Road &s_road, vector<Car>&m_car, Referee &m_referee, bool priority);

void driveCarInitList(bool priority, vector<Road>&m_road, vector<Car>&m_car, Referee &m_referee);

void driveJustCurrentRoad_single_channel(Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee, int channel);
void driveJustCurrentRoad_part(Road &s_road, vector<Car>&m_car, vector<vector<int>> &car_on_road, Referee &m_referee);
void driveJustCurrentRoad(vector<Road>&m_road, vector<Car>&m_car, Referee &m_referee);

bool movToNextRoad(int next_road_index,vector<Car> &m_car,int car_index, vector<Road> &m_road, int cross_index_now, int road_index_now, int S2, int car_x,int car_y, vector<vector<int>> &car_on_road, Referee &m_referee);

bool driveCarInWaitState(vector<Cross> &m_cross, vector<Road>&m_road, vector<Car> &m_car, Referee &m_referee);
int fake_next_road(Road&s_road,Cross&s_cross);
#endif // REFEREE_SYSTEM_H
