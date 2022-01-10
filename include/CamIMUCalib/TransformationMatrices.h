//
// Created by yann on 2022/1/9.
//

#ifndef CAMIMUCALIB_TRANSFORMATIONMATRICES_H
#define CAMIMUCALIB_TRANSFORMATIONMATRICES_H

#include "opencv2/opencv.hpp"


class TransformationMatrices {
public:
    TransformationMatrices() = default;

    bool is_valid() const {return this->valid_flag;}
    void set_valid(bool valid) {this->valid_flag = valid;}

    void set_R_target2cam(const cv::Mat& _R_target2cam) {this->R_target2cam = _R_target2cam;}
    void set_T_target2cam(const cv::Mat& _T_target2cam) {this->T_target2cam = _T_target2cam;}
    void set_R_gripper2base(const cv::Mat& _R_gripper2base) {this->R_gripper2base = _R_gripper2base;}
    void set_T_gripper2base(const cv::Mat& _T_gripper2base) {this->T_gripper2base = _T_gripper2base;}

    cv::Mat get_R_target2cam() const {return this->R_target2cam;}
    cv::Mat get_T_target2cam() const {return this->T_target2cam;}
    cv::Mat get_R_gripper2base() const {return this->R_gripper2base;}
    cv::Mat get_T_gripper2base() const {return this->T_gripper2base;}

    ~TransformationMatrices() = default;

private:
    bool valid_flag;
    cv::Mat R_target2cam;
    cv::Mat T_target2cam;
    cv::Mat R_gripper2base;
    cv::Mat T_gripper2base;
};


#endif //CAMIMUCALIB_TRANSFORMATIONMATRICES_H
