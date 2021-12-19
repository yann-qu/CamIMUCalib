
#include "CamIMUCalib/CamIMUCalibFun.h"

void Cal_T_gripper2base(std::vector<cv::Mat>& T_gripper2base)
{
    for(auto &i : T_gripper2base){
        i = (cv::Mat_<double>(3,1) << 0, 0, 0);
    }
}

void Cal_R_gripper2base(std::vector<cv::Mat>& R_gripper2base)
{
    /**
     * pitch - theta_x
     * yaw - theta_z
     * row - theta_y
     */
    double yaw, pitch, roll, theta_x, theta_y, theta_z;
    for(int i = 0; i < R_gripper2base.size(); i++){
        std::ifstream inputFile("../data/IMU/"+std::to_string(i)+".txt", std::ifstream::in);
        std::string line, word;
        std::getline(inputFile, line);
        std::stringstream ss(line);
        ss >> word;
        yaw = atof(word.c_str());
        ss >> word;
        pitch = atof(word.c_str());
        ss >> word;
        roll = atof(word.c_str());

        theta_x = pitch * PI / 180.0;
        theta_y = -roll * PI / 180.0;
        theta_z = yaw * PI / 180.0;
//        theta_x =  pitch* PI / 180.0;
//        theta_y =  - yaw * PI / 180.0;
//        theta_z =  - roll * PI / 180.0;
//        theta_x = pitch * PI / 180.0;
//        theta_y = yaw * PI / 180.0;
//        theta_z = roll * PI / 180.0;

        cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
        cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
        cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);
        cv::Mat R = R_z * R_y * R_x;
//        cv::invert(R, R_gripper2base[i]);
        R_gripper2base[i] = R;
    }
}


void Cal_R_T_target2cam(std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& T_target2cam)
{
    for(int i = 0; i < dataNum; i++){
        cv::Mat rvec, tvec, rotationMat;
        std::vector<cv::Point3d> objectPoints;
        for(int row = 0; row < patternSize.height; row++){
            for(int col = 0; col < patternSize.width; col++){
                objectPoints.push_back(cv::Point3d(25.0 * col, 25.0 * row, 0));
            }
        }
        cv::Mat src = cv::imread("../data/img/" + std::to_string(i) + ".jpg", cv::IMREAD_COLOR);
        cv::Mat corners;
        bool found = cv::findChessboardCorners(src, patternSize, corners);
        cv::drawChessboardCorners(src, patternSize, corners, found);
        cv::imshow("src with corners", src);
        cv::waitKey(0);
        cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
        cv::Rodrigues(rvec, rotationMat);
        T_target2cam[i] = tvec;
        R_target2cam[i] = rotationMat;
    }
}


