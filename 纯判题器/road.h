#ifndef ROAD_H
#define ROAD_H

#include<vector>

class Road
{
public:
    //#(id,     length,  speed, channel,    from,    to,  isDuplex)
    int road_id,length,speedlim, rwidth,start_ind,end_ind,isDuplex;

    int road_index;
    std::vector<int> carport_from;//该条路存放的车的索引
    std::vector<int> carport_to;//该条路存放的车的索引
    std::vector<int> pri_queue_from;
    std::vector<int> pri_queue_to;
    std::vector<std::vector<int>> car_on_road_from;
    std::vector<std::vector<int>> car_on_road_to;
    Road(int rid,int len,int slim,int wth,int sid,int eid,int twy,int index);
    ~Road();
};
#endif // ROAD_H


