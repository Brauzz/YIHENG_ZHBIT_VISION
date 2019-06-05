#pragma once
#include <opencv2/opencv.hpp>
using namespace cv;

class ZeYuPredict
{
public:
    ZeYuPredict(float Qp, float Qv, float Rp,float Rv,float dta, float pre_dta);
    void SetDelta(float dta);
    void SetPreDelta(float pre_dta);
    void ClearFilters(void);
    void InitFilters(float gim_angle);
    void setQRT(int Qp, int Qv, int Rp ,int dta, int pre_dta);
    float run_position(float gim_angle);

    bool exit_flag;
private:
    Mat Qvar;
    Mat Rvar;
    Mat A;
    Mat B;
    Mat H;

    float delta;
    float pre_delta;

    //state
    Mat po;         //error covarience
    Mat po_pre;     //error co
    Mat x_pre;    //the last angle
    Mat kg;       //kalman gain
    Mat xkf;      //kalman angle
    float v;
    float last_v;
    float predict;  //predict angle
};


class PredictMode{
public:
    PredictMode(int _find_size = 5, int _lost_size = 3)
        :find_cnt_size(_find_size),lost_cnt_size(_lost_size),mode(false){}
    bool getResult(bool find_flag)
    {
        if(lost_history.size()<lost_cnt_size)
        {
            lost_history.push_back(find_flag);
        }
        else {
            lost_history.push_back(find_flag);
            lost_history.erase(lost_history.begin());
        }

        if(find_history.size()<find_cnt_size)
        {
            find_history.push_back(find_flag);
        }
        else {
            find_history.push_back(find_flag);
            find_history.erase(find_history.begin());
        }
        std::vector<bool>::iterator iter;
        int lost_cnt = 0, find_cnt = 0;
        for (iter=lost_history.begin();iter!=lost_history.end();iter++){
            if(*iter == 0)
                lost_cnt++;
        }

        for (iter=find_history.begin();iter!=find_history.end();iter++){
            if(*iter == 1)
                find_cnt++;
        }
        if(lost_cnt == lost_cnt_size)
        {
            mode = 0;
        }
        else if(find_cnt == find_cnt_size)
        {
            mode = 1;
        }
        return mode;
    }

private:
    bool mode;
    int lost_cnt_size;
    std::vector<bool> lost_history;

    int find_cnt_size;
    std::vector<bool> find_history;
};
