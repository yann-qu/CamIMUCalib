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

const int dataNum = 114;        // 数据总数
const double chessBoardWidth = 25;   // 标定板棋盘格宽度
cv::Size patternSize(11, 8);   // 标定板内角点数目

// 相机内参
//cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)<<
//                      1267.9470626330208, 0, 290.8667266169044,
//                      0, 1268.795212854859, 348.78536119649493,
//                      0, 0, 1);
//cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
//                     -0.2221014240737975, 0.604040838942106, 0.0017281277134342178, -0.001742049510186971, -2.872820940900385);

// 相机内参
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3)<<
        1275.564865697823, 0, 596.6116807216317,
        0, 1275.0965822787343, 535.9451432294774,
        0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 5) <<
        -0.23757943660630113, 0.26470336920503346, 0.0007890337800371418, -0.000972473387619995, -0.2047925576074175);

int main(int argc, char** argv)
{
    std::vector<TransformationMatrices> T_vec(dataNum);

    // 各个坐标系间的旋转矩阵和平移矩阵
    std::vector<cv::Mat> R_target2cam;
    std::vector<cv::Mat> T_target2cam;
    std::vector<cv::Mat> R_gripper2base;
    std::vector<cv::Mat> T_gripper2base;
    std::vector<cv::Mat> target_coordinates;
    cv::Mat R_cam2gripper, T_cam2gripper;

    Cal_T_gripper2base(T_vec);
    Cal_R_gripper2base(T_vec);
    Cal_R_T_target2cam(T_vec);
    extract_valid_data(T_vec, R_target2cam, T_target2cam,R_gripper2base, T_gripper2base);

    cv::calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper);

    int displayIdx = 10, valid_num;

    std::cout << R_gripper2base[displayIdx] << "\n" << T_gripper2base[displayIdx] << "\n" << R_target2cam[displayIdx] << "\n" << T_target2cam[displayIdx] << std::endl;

    std::cout << "------------" << std::endl;
    std::cout << R_cam2gripper << "\n" << T_cam2gripper << std::endl;

    coordinate_inspection(R_target2cam, T_target2cam, R_cam2gripper, T_cam2gripper, R_gripper2base, T_gripper2base, target_coordinates);

    std::vector<double> n(R_target2cam.size()), x(n.size()), y(n.size()), z(n.size());
    for(int i = 0; i < n.size(); i++) {
        n[i] = i;
        x[i] = target_coordinates[i].at<double>(0);
        y[i] = target_coordinates[i].at<double>(1);
        z[i] = target_coordinates[i].at<double>(2);
    }
    auto axes = CvPlot::makePlotAxes();
    axes.create<CvPlot::Series>(n, x, "-g");
    axes.create<CvPlot::Series>(n, y, "-b");
    axes.create<CvPlot::Series>(n, z, "-r");
    CvPlot::show("mywindow", axes);

    return 0;
}



