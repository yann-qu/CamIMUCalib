//
// Created by yann on 2021/12/11.
//

#ifndef CAMIMUCALIB_CAMIMUCALIBFUN_H
#define CAMIMUCALIB_CAMIMUCALIBFUN_H

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include "opencv2/opencv.hpp"

extern const int dataNum;
extern cv::Size patternSize;
extern cv::Mat cameraMatrix;
extern cv::Mat distCoeffs;
const double PI = 3.1415926535;

void Cal_T_gripper2base(std::vector<cv::Mat>& T_gripper2base);
void Cal_R_gripper2base(std::vector<cv::Mat>& R_gripper2base);
void Cal_R_T_target2cam(std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& T_target2cam);

#endif //CAMIMUCALIB_CAMIMUCALIBFUN_H
