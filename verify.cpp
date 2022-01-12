/**
 * @brief 获取相机与IMU标定的数据，即图像数据和对应的IMU数据
 * @brief 相机为海康工业相机 25mm
 */

#include <iostream>
#include <string>
#include <glog/logging.h>
#include "opencv2/opencv.hpp"
#include "Camera/hk_cam_wrapper.h"
#include "com/rmserial.h"
#include "CamIMUCalib/TransformationMatrices.h"
#include "CamIMUCalib/CamIMUCalibFun.h"


const double chessBoardWidth = 25;   // 标定板棋盘格宽度
cv::Size patternSize(11, 8);   // 标定板内角点数目

std::string uart_port = "/dev/ttyACM0";
RmSerial rmSerial;
Camera *cam = nullptr;
cv::Mat src;
std::string camera_sn = "00F78889001";
// 1280 * 1024 MV-CA013-21UC
//int roi_offset_x = 320, roi_offset_y = 192, roi_width = 640, roi_height = 640;
int roi_offset_x = 0, roi_offset_y = 0, roi_width = 1024, roi_height = 1024;
//int roi_offset_x = 160, roi_offset_y = 96, roi_width = 960, roi_height = 832;
float ARMOR_CAMERA_EXPOSURE = 10000, ARMOR_CAMERA_GAIN = 6;
bool keepRunning = true;

// 相机内参
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)<<
        1275.564865697823, 0, 596.6116807216317,
        0, 1275.0965822787343, 535.9451432294774,
        0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.23757943660630113, 0.26470336920503346, 0.0007890337800371418, -0.000972473387619995, -0.2047925576074175);

cv::Mat R_cam2gripper = (cv::Mat_<double>(3, 3) <<
        0.9995450586393724, 0.0004824436639992419, -0.03015697262200431,
        0.03014348777397187, 0.01792430953258028, 0.9993848554348822,
        0.001022689803243715, -0.9998392302644916, 0.01790161248812594);

cv::Mat T_cam2gripper = (cv::Mat_<double>(3, 1) <<
        -7.484137864602875,
        68.7858983235629,
        112.0455164370746);

static void OnInit()
{
    rmSerial.init();
    src = cv::Mat(1024, 1024, CV_8UC3);
    cam = new HKCamera(camera_sn);

    cam->init(roi_offset_x,
              roi_offset_y, roi_width,
              roi_height, ARMOR_CAMERA_EXPOSURE,
              ARMOR_CAMERA_GAIN, true);

    if (!cam->init_is_successful()) {
        LOG(ERROR) << "Camera Init Failed!";
        keepRunning = false;
        return;
    }
    cam->start();
}

static void OnClose()
{
    cam->stop();
    if (cam != NULL) delete cam;
}

int main(int argc, char **argv)
{
    int index = 0, key;
    double yaw, roll, pitch;
    std::vector<TransformationMatrices> T_vec; // 存储点的相关矩阵

    OnInit();

    std::vector<double> n, x, y, z; // 存储目标坐标数据点

    while (keepRunning) {
        TransformationMatrices T;
        cv::Mat target_coordinate;
        LOG(INFO) << "Recieve Mcu IMU Data:" << "yaw:" << receive_config_data.yaw
                  << "\tpitch:" << receive_config_data.pitch
                  << "\troll:" << receive_config_data.roll << std::endl;

        if (key == 'q' || key == 27) keepRunning = false;

        cam->read(src);
        yaw = receive_config_data.yaw;
        roll = receive_config_data.roll;
        pitch = receive_config_data.pitch;

        Cal_T_gripper2base(T);
        Cal_R_gripper2base(T, yaw, roll, pitch);
        Cal_R_T_target2cam(T, src);

        cv::imshow("src", src);
        key = cv::waitKey(33);

        if(!T.is_valid()) continue;

        coordinate_inspection(T, R_cam2gripper, T_cam2gripper, target_coordinate);

        n.push_back(index);
        x.push_back(target_coordinate.at<double>(0));
        y.push_back(target_coordinate.at<double>(1));
        z.push_back(target_coordinate.at<double>(2));

        auto axes = CvPlot::makePlotAxes();
        axes.create<CvPlot::Series>(n, x, "-g");
        axes.create<CvPlot::Series>(n, y, "-b");
        axes.create<CvPlot::Series>(n, z, "-r");
        cv::Mat mat = axes.render(300, 400);
        cv::imshow("Target Point Coordinate", mat);

        key = cv::waitKey(33);
        index++;
    }
    OnClose();
    return 0;
}


