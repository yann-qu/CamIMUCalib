
#include "CamIMUCalib/CamIMUCalibFun.h"

void Cal_T_gripper2base(std::vector<TransformationMatrices>& T_vec)
{
    for(auto &i : T_vec){
        i.set_T_gripper2base((cv::Mat_<double>(3,1) << 0, 0, 0));
    }
}

void Cal_T_gripper2base(TransformationMatrices& T)
{
    T.set_T_gripper2base((cv::Mat_<double>(3,1) << 0, 0, 0));
}

void Cal_R_gripper2base(std::vector<TransformationMatrices>& T_vec)
{
    /**
     * pitch - theta_x
     * yaw - theta_z
     * row - theta_y
     */
    double yaw, pitch, roll, theta_x, theta_y, theta_z;
    for(int i = 0; i < T_vec.size(); i++){
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

        cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
        cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
        cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);
        cv::Mat R = R_z * R_y * R_x;
//        cv::invert(R, R_gripper2base[i]);
        T_vec[i].set_R_gripper2base(R);
    }
}

void Cal_R_gripper2base(TransformationMatrices& T, double yaw, double roll, double pitch)
{
    double theta_x, theta_y, theta_z;
    theta_x = pitch * PI / 180.0;
    theta_y = -roll * PI / 180.0;
    theta_z = yaw * PI / 180.0;
    cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(theta_x), -sin(theta_x), 0, sin(theta_x), cos(theta_x));
    cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(theta_y), 0, sin(theta_y), 0, 1, 0, -sin(theta_y), 0, cos(theta_y));
    cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(theta_z), -sin(theta_z), 0, sin(theta_z), cos(theta_z), 0, 0, 0, 1);
    cv::Mat R = R_z * R_y * R_x;
    T.set_R_gripper2base(R);
}

void Cal_R_T_target2cam(std::vector<TransformationMatrices>& T_vec)
{
    for(int i = 0; i < T_vec.size(); i++){
        cv::Mat rvec, tvec, rotationMat;
        std::vector<cv::Point3d> objectPoints;
        for(int row = 0; row < patternSize.height; row++){
            for(int col = 0; col < patternSize.width; col++){
                objectPoints.push_back(cv::Point3d(chessBoardWidth * col, chessBoardWidth * row, 0));
            }
        }
        cv::Mat src = cv::imread("../data/img/" + std::to_string(i) + ".jpg", cv::IMREAD_COLOR);
        cv::Mat corners;
        bool found = cv::findChessboardCornersSB(src, patternSize, corners);
        T_vec[i].set_valid(found);
        cv::drawChessboardCorners(src, patternSize, corners, found);
        cv::imshow("src with corners", src);
        cv::waitKey(20);
        if(!found) continue;
        cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
        cv::Rodrigues(rvec, rotationMat);
        T_vec[i].set_T_target2cam(tvec);
        T_vec[i].set_R_target2cam(rotationMat);
    }
}

void Cal_R_T_target2cam(TransformationMatrices& T, cv::Mat& src)
{
    cv::Mat rvec, tvec, rotationMat;
    std::vector<cv::Point3d> objectPoints;
    for(int row = 0; row < patternSize.height; row++){
        for(int col = 0; col < patternSize.width; col++){
            objectPoints.push_back(cv::Point3d(chessBoardWidth * col, chessBoardWidth * row, 0));
        }
    }
    cv::Mat corners;
    bool found = cv::findChessboardCornersSB(src, patternSize, corners);
    T.set_valid(found);
    cv::drawChessboardCorners(src, patternSize, corners, found);
    cv::imshow("src with corners", src);
//    cv::waitKey(20);
    if(!found) return;
    cv::solvePnP(objectPoints, corners, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Rodrigues(rvec, rotationMat);
    T.set_T_target2cam(tvec);
    T.set_R_target2cam(rotationMat);
}

void coordinate_inspection(const std::vector<cv::Mat>& R_target2cam, const std::vector<cv::Mat>& T_target2cam, const cv::Mat& R_cam2gripper, const cv::Mat& T_cam2gripper, const std::vector<cv::Mat>& R_gripper2base, const std::vector<cv::Mat>& T_gripper2base, std::vector<cv::Mat>& target_coordinates)
{
    cv::Mat P_TargetCoordinate = (cv::Mat_<double>(3, 1) << 0, 0, 0 );
    for(int i = 0; i < R_target2cam.size(); i++) {
        target_coordinates.push_back(R_gripper2base[i] * (R_cam2gripper * (R_target2cam[i] * P_TargetCoordinate + T_target2cam[i]) + T_cam2gripper) + T_gripper2base[i]);
    }
}

void coordinate_inspection(const TransformationMatrices& T, const cv::Mat& R_cam2gripper, const cv::Mat& T_cam2gripper,cv::Mat& target_coordinate)
{
    cv::Mat P_TargetCoordinate = (cv::Mat_<double>(3, 1) << 0, 0, 0 );
    target_coordinate = T.get_R_gripper2base() * (R_cam2gripper * (T.get_R_target2cam() * P_TargetCoordinate + T.get_T_target2cam()) + T_cam2gripper) + T.get_T_gripper2base();
}

void extract_valid_data(std::vector<TransformationMatrices>& T_vec, std::vector<cv::Mat>& R_target2cam, std::vector<cv::Mat>& T_target2cam, std::vector<cv::Mat>& R_gripper2base, std::vector<cv::Mat>& T_gripper2base)
{
    for(auto& i : T_vec){
        if(i.is_valid()){
            R_target2cam.push_back(i.get_R_target2cam());
            T_target2cam.push_back(i.get_T_target2cam());
            R_gripper2base.push_back(i.get_R_gripper2base());
            T_gripper2base.push_back(i.get_T_gripper2base());
        }
    }
}
