#ifndef SOLVE_ANGLE_H
#define SOLVE_ANGLE_H
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
class SolveAngle
{
public:
    SolveAngle();
    SolveAngle(const char* file_path, float c_x, float c_y, float c_z, float barrel_y);
    void getAngle(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist, float &theta_y);
    // ---------ICRA--------------------
    void getAngle_ICRA(vector<Point2f>& image_point, float ballet_speed, float& angle_x, float& angle_y, float &dist);
    float GetPitch_ICRA(float x, float y, float v);
    float BulletModel_ICRA(float x, float v, float angle);
    // ---------/ICRA-------------------
    void Generate3DPoints(uint8_t mode, Point2f offset_point);
    Mat cameraMatrix, distCoeffs;
    Mat object_point_mat;
    vector<Point3f> objectPoints;
    vector<Point2f> projectedPoints;
    vector<Point2f> imagePoints;
    Mat rvec;
    Mat tvec;
    float fx;
    float fy;
    float height_world = 60.0;
    float overlap_dist = 100000.0;
    float barrel_ptz_offset_y = -0; // mm   + ptz is up barrel
    float ptz_camera_x = 0;       // +left
    float ptz_camera_y = 52.5;       // + camera is  ptz
    float ptz_camera_z = -135;//-225;     // - camera is front ptz
    float scale = 1.3;              // is calc distance scale not use pnp ,test
};

void CodeRotateByZ(double x, double y, double thetaz, double& outx, double& outy);
void CodeRotateByY(double x, double z, double thetay, double& outx, double& outz);
void CodeRotateByX(double y, double z, double thetax, double& outy, double& outz);

#endif // SOLVE_ANGLE_H
