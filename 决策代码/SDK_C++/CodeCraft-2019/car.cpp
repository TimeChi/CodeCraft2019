#include "car.h"

Car::Car(int cid, int sid,int eid,int smax,int gt,int pro,int pre,int ii):car_id(cid),start_ind(sid),end_ind(eid),speedmax(smax),plantime(gt),priority(pro),preset(pre),car_index(ii)
{
   // cout<<"Car_Init_Successful"<<endl;
        speed_now = speedmax;
        state = -1;
}

Car::~Car()
{

}


