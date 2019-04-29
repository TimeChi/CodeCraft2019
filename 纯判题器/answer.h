#ifndef ANSWER_H
#define ANSWER_H
#include <vector>
#include "car.h"
class Answer
{
public:
    int car_id;
    int gotime;
    int car_index;
    std::vector<int> road_index;//路径在m_road中的索引
    std::vector<int> car_path;
    Answer(int cd, int ge, std::vector<int> ch)
    {
        car_id = cd;
        gotime = ge;
        car_path = ch;
    }
};
#endif // ANSWER_H
