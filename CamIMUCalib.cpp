/**
 * @brief 标定。计算转换矩阵
 * @author Yann
 *
 * img -> target2cam
 *    旋转矩阵，平移矩阵
 * IMU -> gripper2base
 *    只有旋转，没有平移
 *
 * 规定坐标系：
 * 过转轴沿炮管方向为y z轴指向上方的右手系
 *
 * 规定base系：
 *
 * 规定gripper系：
 *
 *
 *
 */



//%-- Focal length:
//fc = [ 1267.384876323977323 ; 1265.937472120507664 ];
//
//%-- Principal point:
//cc = [ 601.501632859157326 ; 497.523757835385197 ];
//
//%-- Skew coefficient:
//alpha_c = 0.000000000000000;
//
//%-- Distortion coefficients:
//kc = [ -0.205158318167005 ; 0.149992697867292 ; -0.004128390837994 ; 0.000349383607055 ; 0.000000000000000 ];
//
//%-- Focal length uncertainty:
//fc_error = [ 5.397391037574073 ; 4.934578180870288 ];
//
//%-- Principal point uncertainty:
//cc_error = [ 8.398219891418995 ; 7.570179533251755 ];
//
//%-- Skew coefficient uncertainty:
//alpha_c_error = 0.000000000000000;
//
//%-- Distortion coefficients uncertainty:
//kc_error = [ 0.005962335208084 ; 0.008810279461824 ; 0.001068209155417 ; 0.001006806324659 ; 0.000000000000000 ];



//mtx=np.array( [[1267.9470626330208, 0.0, 290.8667266169044], [0.0, 1268.795212854859, 348.78536119649493], [0.0, 0.0, 1.0]] )
//dist=np.array( [[-0.2221014240737975], [0.604040838942106], [0.0017281277134342178], [-0.001742049510186971], [-2.872820940900385]] )
//K=np.array( [[1262.7544281532205, 0.0, 302.1765800786079], [0.0, 1263.600799647081, 344.64269931903726], [0.0, 0.0, 1.0]] )
//D=np.array( [[0.09974702122997715], [1.8592907288940705], [-42.05203584726265], [340.2544665375835]] )
//total error:  0.014714436342232077


#include "CamIMUCalib/CamIMUCalibFun.h"

// 数据总数
const int dataNum = 41;

// 标定板内角点数目
cv::Size patternSize(11, 8);

// 标定板棋盘格宽度
const int chessWidth = 25;

// 各个坐标系间的旋转矩阵和平移矩阵
std::vector<cv::Mat> R_target2cam(dataNum);
std::vector<cv::Mat> T_target2cam(dataNum);
std::vector<cv::Mat> R_gripper2base(dataNum);
std::vector<cv::Mat> T_gripper2base(dataNum);
cv::Mat R_cam2gripper, T_cam2gripper;

// 相机内参
//cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)<<
//                      1267.9470626330208, 0, 290.8667266169044,
//                      0, 1268.795212854859, 348.78536119649493,
//                      0, 0, 1);
//cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
//                     -0.2221014240737975, 0.604040838942106, 0.0017281277134342178, -0.001742049510186971, -2.872820940900385);

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)<<
                                              1277.0200144834782, 0, 611.8659285614318,
        0, 1276.2730202845898, 544.8178789397944,
        0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
                                             -0.2289201032944158, 0.1980057040105881, 0.000803253302737744, -0.00013540482532685213, -0.03973764847083225);

int main(int argc, char** argv)
{
    int displayIdx = 10;

    Cal_T_gripper2base(T_gripper2base);
    std::cout << T_gripper2base[displayIdx] << std::endl;

    Cal_R_gripper2base(R_gripper2base);
    std::cout << R_gripper2base[displayIdx] << std::endl;

    Cal_R_T_target2cam(R_target2cam, T_target2cam);
    std::cout << T_target2cam[displayIdx] << std::endl;
    std::cout << R_target2cam[displayIdx] << std::endl;

    cv::calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper);

    std::cout << "------------" << std::endl;
    std::cout << R_cam2gripper << std::endl;
    std::cout << T_cam2gripper << std::endl;

    return 0;
}



