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

extern const int dataNum;    // 数据总数
extern const double chessBoardWidth; // 标定板角点间距
extern cv::Size patternSize; // 标定板内角点数目
extern cv::Mat cameraMatrix; // 相机内参
extern cv::Mat distCoeffs;   // 相机内参
const double PI = 3.1415926535;

/**
 * @brief 计算gripper2base T
 * @param T_vec 存储TransformationMatrices的向量。
 * @attention 适用于从文件中读取数据进行标定的场景。
 */
void Cal_T_gripper2base(std::vector<TransformationMatrices>& T_vec);

/**
 * @brief 计算gripper2base T
 * @param T TransformationMatrices
 * @attention 适用于单张图片实时计算的场景。
 */
void Cal_T_gripper2base(TransformationMatrices& T);

/**
 * @brief 计算gripper2base R
 * @param T_vec 存储TransformationMatrices的向量。
 * @attention 适用于从文件中读取数据进行标定的场景。
 */
void Cal_R_gripper2base(std::vector<TransformationMatrices>& T_vec);

/**
 * @brief 计算gripper2base R
 * @param T TransformationMatrices
 * @param yaw 当前帧图片对应的yaw轴角度
 * @param roll 当前帧图片对应的roll轴角度
 * @param pitch 当前帧图片对应的pitch轴角度
 * @brief 适用于单张图片实时计算的场景。
 */
void Cal_R_gripper2base(TransformationMatrices& T, double yaw, double roll, double pitch);

/**
 * @brief 计算target2cam R T
 * @param T_vec 存储TransformationMatrices的向量。
 * @attention 适用于从文件中读取数据进行标定的场景。
 */
void Cal_R_T_target2cam(std::vector<TransformationMatrices>& T_vec);

/**
 * @brief 计算target2cam R T
 * @param T TransformationMatrices
 * @param src 当前帧图片
 * @attention 适用于单张图片实时计算的场景。
 */
void Cal_R_T_target2cam(TransformationMatrices& T, cv::Mat& src);

/**
 * @brief 计算目标坐标值，检验算法
 * @param R_target2cam
 * @param T_target2cam
 * @param R_cam2gripper
 * @param T_cam2gripper
 * @param R_gripper2base
 * @param T_gripper2base
 * @param target_coordinates
 * @attention 适用于从文件中读取数据进行标定的场景。
 */
void coordinate_inspection(const std::vector<cv::Mat>& R_target2cam, const std::vector<cv::Mat>& T_target2cam, const cv::Mat& R_cam2gripper, const cv::Mat& T_cam2gripper, const std::vector<cv::Mat>& R_gripper2base, const std::vector<cv::Mat>& T_gripper2base, std::vector<cv::Mat>& target_coordinates);

/**
 * @brief 计算目标坐标值，检验算法
 * @param T
 * @param R_cam2gripper
 * @param T_cam2gripper
 * @param target_coordinate
 * @attention 适用于单张图片实时计算的场景。
 */
void coordinate_inspection(const TransformationMatrices& T, const cv::Mat& R_cam2gripper, const cv::Mat& T_cam2gripper,cv::Mat& target_coordinate);

/**
 * @brief 从存储TransformationMatrices的向量中提取合法是数据
 * @param T_vec
 * @param R_target2cam
 * @param T_target2cam
 * @param R_gripper2base
 * @param T_gripper2base
 * @attention 适用于从文件中读取数据进行标定的场景。
 */
void extract_valid_data(std::vector<TransformationMatrices>& T_vec, std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& T_target2cam, std::vector<cv::Mat>& R_gripper2base, std::vector<cv::Mat>& T_gripper2base); // 提取有效数据

#endif //CAMIMUCALIB_CAMIMUCALIBFUN_H
