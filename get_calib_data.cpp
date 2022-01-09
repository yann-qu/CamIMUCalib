/**
 * @brief 获取相机与IMU标定的数据，即图像数据和对应的IMU数据
 * @brief 相机为海康工业相机 25mm
 */

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <glog/logging.h>
#include "opencv2/opencv.hpp"
#include "Camera/hk_cam_wrapper.h"
#include "com/rmserial.h"

// 0: 采集Cam内参标定数据
// 1: 采集CamIMU标定数据
const int dataFlag = 1;

std::string uart_port = "/dev/ttyACM0";
RmSerial rmSerial;
Camera* cam = nullptr;
cv::Mat src;
std::string camera_sn = "00F78889001";
int key;
// 1280 * 1024 MV-CA013-21UC
//int roi_offset_x = 320, roi_offset_y = 192, roi_width = 640, roi_height = 640;
int roi_offset_x = 0, roi_offset_y = 0, roi_width = 1024, roi_height = 1024;
//int roi_offset_x = 160, roi_offset_y = 96, roi_width = 960, roi_height = 832;
float ARMOR_CAMERA_EXPOSURE = 10000, ARMOR_CAMERA_GAIN = 15;
bool keepRunning = true;

static void OnInit() {
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

static void OnClose() {
    cam->stop();
    if (cam != NULL) delete cam;
}

int main(int argc, char** argv)
{
    OnInit();
    int start_index = 0;

    while (keepRunning){
        cam->read(src);

        cv::imshow("src", src);
        key = cv::waitKey(33);

        if(dataFlag == 0){
            if(key == 'q'  || key == 27) keepRunning = false;
            else if(key == 's') {
                std::ostringstream buffer;

                buffer << "../CamCalib/img/" << start_index << ".jpg";
                cv::imwrite(buffer.str(), src);
                std::cout << buffer.str() << " is saved" << std::endl;
                buffer.str("");

                start_index ++;
            }
        }
        else if(dataFlag == 1){
            LOG(INFO) << "Recieve Mcu IMU Data:"<< "yaw:" << receive_config_data.yaw
                      << "\tpitch:" << receive_config_data.pitch
                      << "\troll:" << receive_config_data.roll << std::endl;

            if(key == 'q'  || key == 27) keepRunning = false;
            else if(key == 's') {
                std::ostringstream buffer;

                buffer << "../data/img/" << start_index << ".jpg";
                cv::imwrite(buffer.str(), src);
                std::cout << buffer.str() << " is saved" << std::endl;
                buffer.str("");

                buffer << "../data/IMU/" << start_index << ".txt";
                std::ofstream IMU_file(buffer.str());
                if( !IMU_file.is_open()) {
                    std::cerr << "Error opening file\n";
                    exit(-1);
                } else {
                    IMU_file << receive_config_data.yaw << " " << receive_config_data.pitch << " " << receive_config_data.roll;
                    IMU_file.close();
                    std::cout << buffer.str() << " is saved" << std::endl;
                }

                start_index ++;
            }
        }

    }

    OnClose();
    return 0;
}


