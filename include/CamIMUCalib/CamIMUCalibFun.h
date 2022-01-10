//
// Created by yann on 2021/12/11.
//

#ifndef CAMIMUCALIB_CAMIMUCALIBFUN_H
#define CAMIMUCALIB_CAMIMUCALIBFUN_H

#define CVPLOT_HEADER_ONLY

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include "opencv2/opencv.hpp"
#include "CvPlot/cvplot.h"
#include "CamIMUCalib/TransformationMatrices.h"

extern const int dataNum;
extern cv::Size patternSize;
extern cv::Mat cameraMatrix;
extern cv::Mat distCoeffs;
const double PI = 3.1415926535;

void Cal_T_gripper2base(std::vector<TransformationMatrices>& T_vec); // 计算gripper2base T
void Cal_R_gripper2base(std::vector<TransformationMatrices>& T_vec); // 计算gripper2base R
void Cal_R_T_target2cam(std::vector<TransformationMatrices>& T_vec); // 计算target2cam R T
void coordinate_inspection(const std::vector<cv::Mat>& R_target2cam, const std::vector<cv::Mat>& T_target2cam, const cv::Mat& R_cam2gripper, const cv::Mat& T_cam2gripper, const std::vector<cv::Mat>& R_gripper2base, const std::vector<cv::Mat>& T_gripper2base, std::vector<cv::Mat>& target_coordinates); // 计算目标坐标值，检验算法
void extract_valid_data(std::vector<TransformationMatrices>& T_vec, std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& T_target2cam, std::vector<cv::Mat>& R_gripper2base, std::vector<cv::Mat>& T_gripper2base); // 提取有效数据

#endif //CAMIMUCALIB_CAMIMUCALIBFUN_H
