#ifndef CROSS_H
#define CROSS_H
#include <vector>
using namespace std;
#include<algorithm>
class Cross
{
public:
    int cross_id;//路口id
    int cross_index;//路口在m_cross中的索引
    int road_north_id;//北方向道路id
    int road_east_id;//东方向道路id
    int road_south_id;//南方向道路id
    int road_west_id;//西方向道路id
    std::vector<int> cross_road_rank;//存放的是道路对应的序号，顺序与cross_road_sort里的道路一一对应
    std::vector<int> cross_road_sort;//存放的是排好序后的道路id
    std::vector<int> cross_road_index;//对应cross_road_sort里的每个道路在m_road里的index
    Cross(int nid,int rn,int re,int rs,int rw,int index);
    ~Cross();
};

void QuickSort(vector<int> &arr, vector<int> &Carid,int first, int end);
#endif // CROSS_H
