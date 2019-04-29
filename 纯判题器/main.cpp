#include "referee_system.h"

int main()
{

    std::cout << "Begin" << std::endl;

    std::string carPath("../config/car.txt");//路径需要改
    std::string roadPath("../config/road.txt");
    std::string crossPath("../config/cross.txt");
    std::string answerPath("../config/answer.txt");
    std::string PanswerPath("../config/presetAnswer.txt");

    std::cout << "carPath is " << carPath << std::endl;
    std::cout << "roadPath is " << roadPath << std::endl;
    std::cout << "crossPath is " << crossPath << std::endl;
    std::cout << "answerPath is " << answerPath << std::endl;

    //计时，查看系统运行时间
    time_t begin,end;
    double ret;
    begin = clock();
    //数据初始化
    vector<Car> m_car;
    vector<Road> m_road;
    vector<Cross> m_cross;
    vector<presetCar> m_pcar;
    vector<Answer> m_answer;
    map<int, int> cross_map;
    map<int, int> road_map;
    map<int, int> car_map;
    Data_init(m_car,m_road,m_cross,m_answer,m_pcar,carPath,roadPath,crossPath,answerPath,PanswerPath);
    Init_all(m_answer,m_pcar,m_cross,m_road,m_car,cross_map,road_map,car_map);
    if(!referee_machine(m_cross,m_road,m_car))
    {
        cout<<"death_lock"<<endl;
    }

    //测试代码运行时间
    end = clock();
    ret=double(end-begin)/CLOCKS_PER_SEC;
    cout<<"runtime:   "<<ret<<" s"<<endl;

    return 0;
}



