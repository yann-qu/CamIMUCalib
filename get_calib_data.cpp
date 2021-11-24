/**
 * @brief 获取相机与IMU标定的数据，即图像数据和对应的IMU数据
 * @brief 相机为海康工业相机
 */

#include <iostream>
#include <string>
#include <glog/logging.h>
#include "opencv2/opencv.hpp"
#include "Camera/hk_cam_wrapper.h"


Camera* cam = nullptr;
cv::Mat src;
std::string camera_sn = "00F78889036";
int key;
int roi_offset_x = 0, roi_offset_y = 0, roi_width = 640, roi_height = 640;
float ARMOR_CAMERA_EXPOSURE = 10000, ARMOR_CAMERA_GAIN = 0;
bool keepRunning = true;

static void OnInit() {
    src = cv::Mat(640, 640, CV_8UC3);
    cam = new HKCamera(camera_sn);

    cam->init(roi_offset_x,
              roi_offset_y, roi_width,
              roi_height, ARMOR_CAMERA_EXPOSURE,
              ARMOR_CAMERA_GAIN, false);

    if (!cam->init_is_successful()) {
        LOG(ERROR) << "Camera Init Failed!";
        keepRunning = false;
        return;
    }
    cam->start();
}

static void OnClose() {
    cam->stop();
    if (cam != NULL) delete cam;
}

int main(int argc, char** argv)
{
    OnInit();
    while (keepRunning){
        cam->read(src);
        // TODO 读取IMU数据

        cv::imshow("src", src);
        key = cv::waitKey(10);
        if(key == 27) keepRunning = false;
        else if(key == 's') {
            // TODO 按下s保存图像和IMU数据到文件
        }
    }



    OnClose();
    return 0;
}


