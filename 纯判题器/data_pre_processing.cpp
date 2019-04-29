#include "data_pre_processing.h"

/*
*函数名称：SplitString
*函数功能：分割字符串
*函数参数：被分割的字符串s 分割结果存储矩阵v 分割间隔符c
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
void SplitString(const string& s, vector<string>& v, const string& c)
{
    string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
}

/*
*函数名称：myReadcross
*函数功能：读入数据，按数据格式放置到m_cross中
*函数参数：显而易见
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
void myReadcar(ifstream &infile,vector<Car> &m_car)
{
    if(!infile.is_open())
        cout<< "Open file failure"<< endl;
    string s;
    int i=0;
    while(getline(infile,s))
    {
        vector<string> v;

        if(s[0]!='#')
        {
            s.erase(s.begin());
            s.erase(s.end()-1);
            SplitString(s, v, ",");
            Car icar(atoi(v[0].c_str()),atoi(v[1].c_str()),atoi(v[2].c_str()),atoi(v[3].c_str()),atoi(v[4].c_str()),atoi(v[5].c_str()),atoi(v[6].c_str()),i);
            m_car.push_back(icar);
            i++;
        }
    }
    infile.close();
}

/*
*函数名称：myReadcross
*函数功能：读入数据，按数据格式放置到m_cross中
*函数参数：显而易见
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
void myReadcross(ifstream &infile, vector<Cross> &m_cross)
{
    if (!infile.is_open())
        cout << "Open file failure" << endl;

    string s;
    int i = 0;
    while (getline(infile, s))
    {
        vector<string> v;

        if (s[0] != '#')
        {
            s.erase(s.begin());
            s.erase(s.end() - 1);
            SplitString(s, v, ",");
            Cross icross(atoi(v[0].c_str()), atoi(v[1].c_str()), atoi(v[2].c_str()), atoi(v[3].c_str()), atoi(v[4].c_str()),i);
            m_cross.push_back(icross);
            i++;
        }
    }
    infile.close();
}

/*
*函数名称：myReadroad
*函数功能：读入数据，按数据格式放置到m_road中
*函数参数：显而易见
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
void myReadroad(ifstream &infile, vector<Road> &m_road)
{
    if (!infile.is_open())
        cout << "Open file failure" << endl;

    string s;
    int i = 0;
    while (getline(infile, s))
    {
        vector<string> v;

        if (s[0] != '#')
        {
            s.erase(s.begin());
            s.erase(s.end() - 1);
            SplitString(s, v, ",");
            Road iroad(atoi(v[0].c_str()), atoi(v[1].c_str()), atoi(v[2].c_str()), atoi(v[3].c_str()), atoi(v[4].c_str()), atoi(v[5].c_str()), atoi(v[6].c_str()),i);
            m_road.push_back(iroad);
            i++;
        }
    }
    infile.close();
}

void mypresetAnswer(ifstream &infile,vector<presetCar> &m_pcar)
{
    if(!infile.is_open())
        cout<< "Open file failure"<< endl;
    string s;
    int i=0;
    while(getline(infile,s))
    {
        vector<string> v;

        if(s[0]!='#')
        {
            s.erase(s.begin());
            s.erase(s.end()-1);
            SplitString(s, v, ",");
            presetCar ipCar(atoi(v[0].c_str()),atoi(v[1].c_str()));
            for(uint i=2; i<v.size(); i++)
            {
                ipCar.path_PA.push_back(atoi(v[i].c_str()));
            }
            m_pcar.push_back(ipCar);
            i++;
        }
    }
    infile.close();
}

void myReadparam(ifstream &infile, vector<int> &N, vector<int> &caronroad_threshold, vector<int> &time_interval)
{
    if (!infile.is_open())
        cout << "Open file failure" << endl;

    string s;
    int i = 0;
    while (getline(infile, s))
    {
        vector<string> v;

        if (s[0] != '#')
        {
            s.erase(s.begin());
            s.erase(s.end() - 1);
            SplitString(s, v, ",");
            N.push_back(atoi(v[0].c_str()));
            caronroad_threshold.push_back(atoi(v[1].c_str()));
            time_interval.push_back(atoi(v[2].c_str()));
            i++;
        }
    }
    infile.close();
}

void myReadanswer(ifstream &infile, vector<Answer> &m_answer)
{
    if (!infile.is_open())
        cout << "Open file failure" << endl;

    string s;
    int i = 0;
    while (getline(infile, s))
    {
        vector<string> v;
        vector<int> path;
        if (s[0] != '#')
        {
            s.erase(s.begin());
            s.erase(s.end() - 1);
            SplitString(s, v, ",");

            for(uint j=2; j<v.size();j++)
            {
                path.push_back(atoi(v[j].c_str()));
            }
            Answer ianswer(atoi(v[0].c_str()),atoi(v[1].c_str()),path);
            m_answer.push_back(ianswer);
            i++;
        }
    }
    infile.close();
}


/*
*函数名称：Data_init
*函数功能：读入数据，分别放置到m_car、m_road、m_cross中
*函数参数：前三个时存储数据的，后三个是文件路径参数
*函数返回值：无
*Author:学学没完
*date：2019.3.14 0:39
*/
void Data_init(vector<Car> &m_car,vector<Road> &m_road,vector<Cross> &m_cross, vector<Answer> &m_answer,vector<presetCar> &m_pcar,string carPath, string roadPath, string crossPath, string answerPath, string PanswerPath)
{
    ifstream infilecar(carPath);
    ifstream infilecross(crossPath);
    ifstream infileroad(roadPath);
    ifstream infileanswer(answerPath);
    ifstream infilePanswer(PanswerPath);
    myReadcar(infilecar, m_car);
    myReadcross(infilecross, m_cross);
    myReadroad(infileroad, m_road);
    mypresetAnswer(infilePanswer,m_pcar);
    myReadanswer(infileanswer,m_answer);
}
