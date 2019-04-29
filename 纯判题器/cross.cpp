#include "cross.h"

Cross::Cross(int nid,int rn,int re,int rs,int rw,int index):cross_id(nid),road_north_id(rn),road_east_id(re),road_south_id(rs),road_west_id(rw),cross_index(index)
{
    cross_road_sort.push_back(rn);
    cross_road_sort.push_back(re);
    cross_road_sort.push_back(rs);
    cross_road_sort.push_back(rw);
    cross_road_rank.push_back(0);
    cross_road_rank.push_back(1);
    cross_road_rank.push_back(2);
    cross_road_rank.push_back(3);
    QuickSort(cross_road_sort, cross_road_rank, 0, cross_road_sort.size()-1);
}

 Cross::~Cross()
{

}

 int OnceSort(vector<int> &arr, vector<int> &Carid,int first, int end)
 {
     int i = first, j = end;
     //当i<j即移动的点还没到中间时循环
     while(i < j)
     {
         //右边区开始，保证i<j并且arr[i]小于或者等于arr[j]的时候就向左遍历
         while(i < j && arr[i] <= arr[j]) --j;
         //这时候已经跳出循环，说明j>i 或者 arr[i]大于arr[j]了，如果i<j那就是arr[i]大于arr[j]，那就交换
         if(i < j)
         {
             int temp = arr[i];
             arr[i] = arr[j];
             arr[j] = temp;
             int temp1 = Carid[i];
             Carid[i] = Carid[j];
             Carid[j] = temp1;
         }
         //对另一边执行同样的操作
         while(i < j && arr[i] <= arr[j]) ++i;
         if(i < j)
         {
             int temp = arr[i];
             arr[i] = arr[j];
             arr[j] = temp;
             int temp1 = Carid[i];
             Carid[i] = Carid[j];
             Carid[j] = temp1;
         }
      }
  //返回已经移动的一边当做下次排序的轴值
  return i;
 }

 /*
 *函数名称：QuickSort
 *函数功能：快速排序 ， 将所有车按运行时间，从小到大进行排序，车的索引放置在Carid中
 *函数参数：arr被排序的数据， Carid存放车索引的矩阵， first开始 ，end结束
 *函数返回值：conflict_flag  1为冲突 0为不冲突
 *Author:学学没完
 *date：2019.3.17 01:10
 */
 void QuickSort(vector<int> &arr, vector<int> &Carid,int first, int end)
 {
     if (first < end)
     {
         int pivot = OnceSort(arr,Carid,first,end);
         //已经有轴值了，再对轴值左右进行递归
         QuickSort(arr,Carid,first,pivot-1);
         QuickSort(arr,Carid,pivot+1,end);
     }
 }
