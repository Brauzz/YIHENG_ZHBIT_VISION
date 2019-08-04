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
#pragma once
#include <opencv2/opencv.hpp>
#include "../../base.h"
#include "../filter/predict.h"

using namespace cv;
using namespace std;
class SolveAngle
{
public:
    SolveAngle(){}
    SolveAngle(const char* file_path, float c_x, float c_y, float c_z, float barrel_y);
    // 普通角度解算
    void getAngle(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist);
    // 能量机关角度解算

    void getBuffAngle(bool flag, vector<Point2f>& image_point, float ballet_speed, float buff_angle, float pre_angle, float gimbal_pitch, float &angle_x, float &angle_y, float &dist);
    float getBuffPitch(float dist, float tvec_y, float ballet_speed);

    // ---------ICRA--------------------
    void getAngle_ICRA(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist);
    float GetPitch_ICRA(float x, float y, float v);
    float BulletModel_ICRA(float x, float v, float angle);
    // ---------/ICRA-------------------
    void Generate3DPoints(uint mode, Point2f offset_point);
    Mat cameraMatrix, distCoeffs;
    Mat object_point_mat;
    vector<Point3f> objectPoints;
    vector<Point2f> projectedPoints;
    vector<Point2f> imagePoints;
    Mat rvec;
    Mat tvec;
    float height_world = 60.0;
    float overlap_dist = 100000.0;
        float barrel_ptz_offset_x = -0;
    float barrel_ptz_offset_y = -0; // mm   + ptz is up barrel

    float ptz_camera_x = 0;       // +left
    float ptz_camera_y = 52.5;       // + camera is  ptz
    float ptz_camera_z = -135;//-225;     // - camera is front ptz
    float scale = 0.99f;              // is calc distance scale not use pnp ,test

    Kalman1 kalman;
    int f_ = 1500;

public:
    float buff_h;

};

class SimpleSolveAngle{
public:
    SimpleSolveAngle(){}
    SimpleSolveAngle(float fx, float fy, float cx, float cy, float f){
        fx_ = fx;
        fy_ = fy;
        cx_ = cx;
        cy_ = cy;
        f_ = f;
    }
    void getAngle(float x_screen, float y_screen, float dh, float &angle_x, float &angle_y, float& distance)
    {
        // 相机坐标系转换
        float camera_xyz[3];
        camera_xyz[2] = height_world * f_ / dh; // Z
        camera_xyz[0] = (x_screen - cx_) * camera_xyz[2] / fx_; // X
        camera_xyz[1] = (y_screen - cy_) * camera_xyz[2] / fy_; // Y
        // 相机坐标系转云台坐标系(不考虑旋转)
        float ptz_xyz[3];
        ptz_xyz[0] = camera_xyz[0] + ptz_camera_x; // X
        ptz_xyz[1] = camera_xyz[1] + ptz_camera_y; // Y
        ptz_xyz[2] = camera_xyz[2] + ptz_camera_z; // Z

        angle_x = atan(ptz_xyz[0] / ptz_xyz[2]);
        angle_x = static_cast<float>(angle_x) * 57.2957805f;
        angle_y = atan(ptz_xyz[1] / ptz_xyz[2]);
        angle_y = static_cast<float>(angle_y) * 57.2957805f;
        distance = ptz_xyz[2];
    }
public:
    float f_ = 1500;

private:
    float fx_;
    float fy_;
    float cx_;
    float cy_;
    float height_world = 60.0;
    float ptz_camera_x = 0;       // +left
    float ptz_camera_y = 52.5;       // + camera is  ptz
    float ptz_camera_z = -135;//-225;     // - camera is front ptz
};

void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz);
void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz);

