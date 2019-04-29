#ifndef DATA_PRE_PROCESSING_H
#define DATA_PRE_PROCESSING_H

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "car.h"
#include "road.h"
#include "cross.h"
#include"answer.h"
#include "presetanswer.h"
#include <algorithm>
#include <time.h>
#include <map>

using namespace std;

void myReadcar(ifstream &infile,vector<Car> &m_car);

void myReadcross(ifstream &infile, vector<Cross> &m_cross);

void myReadroad(ifstream &infile, vector<Road> &m_road);

void Data_init(vector<Car> &m_car,vector<Road> &m_road,vector<Cross> &m_cross, vector<Answer> &m_answer,vector<presetCar> &m_pcar,string carPath, string roadPath, string crossPath, string answerPath, string PanswerPath);

void SplitString(const string& s, vector<string>& v, const string& c);//分解string字符

void myReadparam(ifstream &infile, vector<int> &N, vector<int> &caronroad_threshold, vector<int> &time_interval);

void myReadanswer(ifstream &infile, vector<Answer> &m_answer);

void mypresetAnswer(ifstream &infile,vector<presetCar> &m_pcar);


#endif // DATA_PRE_PROCESSING_H
