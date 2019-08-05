/****************************************************************************
 *  Copyright (C) 2019 cz.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include "solve_angle.h"

using namespace cv;

SolveAngle::SolveAngle(const char* file_path, float c_x, float c_y, float c_z, float barrel_y)
{
    //读取摄像头标定xml文件
    FileStorage fs(file_path, FileStorage::READ);
    // 相关坐标转换偏移数据
    barrel_ptz_offset_y = barrel_y;
    ptz_camera_x = c_x;       // +left
    ptz_camera_y = c_y;       // + camera is  ptz
    ptz_camera_z = c_z;//-225;     // - camera is front ptz
    // 读取相机内参和畸变矩阵
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distCoeffs;
    cout << cameraMatrix << endl;
    cout << distCoeffs << endl;
    Generate3DPoints(0,Point2f(0,0));
    Mat(objectPoints).convertTo(object_point_mat, CV_32F);
    Mat rvec(3, 1, DataType<double>::type);
    Mat tvec(3, 1, DataType<double>::type);
}

void SolveAngle::getAngle(vector<Point2f> &image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist)
{
    // 姿态结算
    solvePnP(objectPoints, image_point, cameraMatrix, distCoeffs, rvec, tvec);
    float dh = ((image_point.at(3).y - image_point.at(0).y) + (image_point.at(2).y - image_point.at(1).y))/2;
    float state_dist = height_world * f_ / dh; // Z
    float final_distance = kalman.merge_run(state_dist, tvec.at<double>(2,0));
    //    tvec.at<double>(2,0)*=scale;
    //    cout << tvec << endl;
    // 估计装甲板y轴坐标旋转量2
    //    double rm[3][3];
    //    Mat rotMat(3, 3, CV_64FC1, rm);
    //    Rodrigues(rvec, rotMat);
    //    theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2])) * 57.2958;
    //    float theta_y = atan2(static_cast<float>(rm[1][0]), static_cast<float>(rm[0][0])) * 57.2958f;//x
    //    theta_y = atan2(-rm[2][0], sqrt(rm[2][0] * rm[2][0] + rm[2][2] * rm[2][2])) * 57.2958;//y
    //    theta_y = atan2(rm[2][1], rm[2][2]) * 57.2958;//z

    // 坐标系转换 -摄像头坐标到云台坐标
    double theta = -atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat position_in_ptz;
    position_in_ptz = r_camera_ptz * tvec - t_camera_ptz;

    //计算子弹下坠补偿
    double bullet_speed = static_cast<double>( ballet_speed);
    const double *_xyz = (const double *)position_in_ptz.data;
    double down_t = 0.0;
    if(bullet_speed > 10e-3)
        down_t = _xyz[2] /1000.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t*down_t * 1000;
#ifdef SET_ZEROS_GRAVITY
    offset_gravity = 0;
#endif
    // 计算角度
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    if(barrel_ptz_offset_y != 0)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(barrel_ptz_offset_y)/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));

        if(xyz[1] < 0)
        {
            Beta = atan(-xyz[1]/xyz[2]);
            angle_y = static_cast<float>(-(alpha+Beta)); //camera coordinate
        }else if(xyz[1] < static_cast<double>(barrel_ptz_offset_y))
        {
            Beta = atan(xyz[1]/xyz[2]);
            angle_y = static_cast<float>(-(alpha - Beta));
        }else
        {
            Beta = atan(xyz[1]/xyz[2]);
            angle_y = static_cast<float>((Beta-alpha));   // camera coordinate
        }
    }else
    {
        angle_y = static_cast<float>(atan2(xyz[1],xyz[2]));
    }
    if(barrel_ptz_offset_x != 0)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(barrel_ptz_offset_x)/sqrt(xyz[0]*xyz[0] + xyz[2]*xyz[2]));
        if(xyz[0] > 0)
        {
            Beta = atan(-xyz[0]/xyz[2]);
            angle_x = static_cast<float>(-(alpha+Beta)); //camera coordinate
        }else if(xyz[0] < static_cast<double>(barrel_ptz_offset_x))
        {
            Beta = atan(xyz[0]/xyz[2]);
            angle_x = static_cast<float>(-(alpha - Beta));
        }else
        {
            Beta = atan(xyz[0]/xyz[2]);
            angle_x = static_cast<float>(Beta-alpha);   // camera coordinate
        }
    }else{
        angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));
    }
    angle_x = static_cast<float>(angle_x) * 57.2957805f;
    angle_y = static_cast<float>(angle_y) * 57.2957805f;
    dist = static_cast<float>(xyz[2]);
}

// ------ 能量机关角度解算 ------

float SolveAngle::GetPitch_ICRA(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float a = 0.0;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 10; i++) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel_ICRA(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
    }
    return a;
}


void SolveAngle::getBuffAngle(bool flag, vector<Point2f> &image_point, float ballet_speed
                              , float buff_angle, float pre_angle, float gimbal_pitch
                              , float &angle_x, float &angle_y, float &dist)
{
    // 姿态结算
    solvePnP(objectPoints, image_point, cameraMatrix, distCoeffs, rvec, tvec);
    // 距离解算 参考能量机关尺寸
    float H = BUFF_H;   // 大能量机关最底部装甲板桥面地面高度
    float h = 430;      // 步兵枪口距离桥面高度mm
    float D = BUFF_DISTANCE;    //步兵距离能量机关水平距离
    float delta_h = H - h;
    float predict_buff_angle = buff_angle + pre_angle;
    buff_h = 800*sin(predict_buff_angle *3.14/180)+800;   // 计算风车相对最底面装甲高度　０－１６００
    float target_h = delta_h + buff_h;
    dist = sqrt(pow(target_h, 2) + pow(D, 2));

    tvec.at<double>(2,0) = dist;

    // 坐标系转换 -摄像头坐标到云台坐标
    double theta = -atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat position_in_ptz;
    position_in_ptz = /*r_camera_ptz **/ tvec - t_camera_ptz;

    const double *_xyz = (const double *)position_in_ptz.data;

    // 计算角度
    double xyz[3] = {_xyz[0], _xyz[1], dist};

    angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));
    angle_x = static_cast<float>(angle_x) * 57.2957805f;
    float gimbal_y = dist * sin(gimbal_pitch*3.14/180);
    float thta = -static_cast<float>(atan2(xyz[1],dist)); // 云台与目标点的相对角度
    float balta = static_cast<float>(atan2(target_h,dist)) - thta; // 云台与地面的相对角度

#ifdef SET_ZEROS_GRAVITY
    angle_y = static_cast<float>(atan2(xyz[1],xyz[2]));
#else

#ifdef USE_GIMBAL_OFFSET
    //    if(flag)
    angle_y = -getBuffPitch(dist/1000, (gimbal_y - xyz[1] )/1000, ballet_speed);
    angle_y += gimbal_pitch*3.1415926f/180;
#else
    //    else
    angle_y = -getBuffPitch(dist/1000, (target_h)/1000, ballet_speed);
    angle_y += balta;
#endif
#endif
    angle_y = static_cast<float>(angle_y) * 57.2957805f ;
}


//--------------------------ICRA------------------------------------
void SolveAngle::getAngle_ICRA(vector<Point2f> &image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist)
{
    Point3f offset_(0, 0, 0);
    float offset_yaw_ = 0.0;
    float offset_pitch_ = 0.0;
    solvePnP(objectPoints, image_point, cameraMatrix, distCoeffs, rvec, tvec);
    Point3f postion = static_cast<Point3f>(tvec);
    angle_x = (float)(atan2(postion.x + offset_.x, postion.z + offset_.z))*180.0f/3.1415926535 + (float)(offset_yaw_);
    angle_y = -GetPitch_ICRA((postion.z + offset_.z) / 1000, -(postion.y + offset_.y) / 1000, ballet_speed)*180.0f/3.1415926535 + (float)(offset_pitch_);
    dist = postion.z;
}

float SolveAngle::BulletModel_ICRA(float x, float v, float angle)
{ //x:m,v:m/s,angle:rad
    float init_k_ = 0.1f;
    float GRAVITY = 9.7887f; //shenzhen 9.7887  zhuhai
    float t, y;
    t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
    y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
    return y;
}


float SolveAngle::getBuffPitch(float dist, float tvec_y, float ballet_speed)
{
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    float GRAVITY = 9.7887f; //shenzhen 9.7887  zhuhai
    y_temp = tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++) {
        // 计算枪口抬升角度
        a = (float) atan2(y_temp, dist);
        // 计算实际落点
        float t, y = 0.0;
        t = dist / (ballet_speed * cos(a));
        y_actual = ballet_speed * sin(a) * t - GRAVITY * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 0.01) {
            break;
        }
    }
    return a;
}

//--------------------------/ICRA------------------------------------

void SolveAngle::Generate3DPoints(uint mode, Point2f offset_point)
{
    objectPoints.clear();
    float x, y, z, width = 0.0, height = 0.0;
    switch (mode) {
    case 1:     // small_armor
        width = 140;//230;//140;      // mm the armor two led width
        height = 60;      // mm the armor led height
        break;
    case 0:     // big_armor
        width = 230;//230;//140;      // mm the armor two led width
        height = 60;      // mm the armor led height
        break;

    case 2:     // buff_armor
        width = 230;//230;//140;      // mm the armor two led width
        height = 130;      // mm the armor led height
        break;
    }

    x = -width/2; y = -height/2; z = 0;
    objectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
    x = width/2; y = -height/2; z = 0;
    objectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
    x = width/2; y = height/2; z = 0;
    objectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
    x = -width/2; y = height/2; z = 0;
    objectPoints.push_back(cv::Point3f(x, y, z)+cv::Point3f(offset_point.x, offset_point.y, 0));
}

void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy)
{
    double x1 = x;
    double y1 = y;
    double rz = thetaz * CV_PI / 180;
    outx = cos(rz) * x1 - sin(rz) * y1;
    outy = sin(rz) * x1 + cos(rz) * y1;
}

void CodeRotateByY(double x, double z, double thetay, double& outx, double &outz)
{
    double x1 = x;
    double z1 = z;
    double ry = thetay * 3.14 / 180;
    outx = cos(ry) * x1 + sin(ry) * z1;
    outz = cos(ry) * z1 - sin(ry) * x1;
}

void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz)
{
    double y1 = y;
    double z1 = z;
    double rx = thetax * 3.14 / 180;
    outy = cos(rx) * y1 - sin(rx) * z1;
    outz = cos(rx) * z1 + sin(rx) * y1;
}
