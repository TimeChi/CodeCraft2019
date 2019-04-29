#include "road.h"
#include <algorithm>

Road::Road(int rid,int len,int slim,int wth,int sid,int eid,int twy,int index):road_id(rid),length(len),speedlim(slim),rwidth(wth),start_ind(sid),end_ind(eid),isDuplex(twy),road_index(index)
{
    std::vector<std::vector<int>> caronroad(rwidth, std::vector<int>(length ,-1));
    car_on_road_from = caronroad;
    car_on_road_to = caronroad;
}

Road::~Road()
{

}



