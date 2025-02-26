#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

int main()
{
    // 定义ArUco字典和参数
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    // 读取相机标定参数
    //cv::FileStorage fs("calibration.xml", cv::FileStorage::READ);
    //cv::Mat cameraMatrix, distCoeffs;
    //fs["camera_matrix"] >> cameraMatrix;
    //fs["distortion_coefficients"] >> distCoeffs;
    double fx = 406.932130;
    double fy = 402.678201;
    double cx = 316.629381;
    double cy = 242.533947;

    double k1 = 0.039106;
    double k2 = -0.056494;
    double p1 = -0.000824;
    double p2 = 0.092161;
    double k3 = 0.0;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
    fx, 0, cx,
    0, fy, cy,
    0, 0, 1);

    cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);


    // 打开相机
    cv::VideoCapture cap(2);
    if (!cap.isOpened())
    {
        std::cout << "无法打开相机" << std::endl;
        return -1;
    }

    while (true)
    {
        // 读取相机帧
        cv::Mat frame;
        cap >> frame;

        // 转换为灰度图像，非必要
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // 检测ArUco二维码
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::detectMarkers(gray, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
        //cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

        // 绘制检测结果
        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            // 估计相机姿态
            /*********************************************************
            void cv::aruco::estimatePoseSingleMarkers(
                const std::vector<std::vector<cv::Point2f>>& corners,  // 检测到的 ArUco 标记角点的二维图像坐标
                float markerLength,                                      // ArUco 标记的边长（单位任意，但与标定过程中的相机参数保持一致）
                const cv::Mat& cameraMatrix,                             // 相机的内参数矩阵
                const cv::Mat& distCoeffs,                               // 相机的畸变参数
                std::vector<cv::Vec3d>& rvecs,                           // 输出的每个标记的旋转向量
                std::vector<cv::Vec3d>& tvecs                            // 输出的每个标记的平移向量
            );
            *********************************************************/
            std::vector<cv::Vec3d> rvecs, tvecs;
            float arucoLength = 0.05; //aruco二维码边长
            cv::aruco::estimatePoseSingleMarkers(markerCorners, arucoLength, cameraMatrix, distCoeffs, rvecs, tvecs);
            for (int i = 0; i < markerIds.size(); ++i)
            {
              cv::Vec3d rvec = rvecs[i];
              cv::Vec3d tvec = tvecs[i];

              std::cout << "Marker ID: " << markerIds[i] << std::endl;
              std::cout << "rvec: " << rvec << std::endl;
              std::cout << "tvec: " << tvec << std::endl;
            }

            // 绘制相对位姿
            for (int i = 0; i < markerIds.size(); ++i)
            {
                cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            }
        }

        // 显示图像
        cv::imshow("ArUco Detection", frame);

        // 按下 'q' 键退出循环
        if (cv::waitKey(1) == 'q')
        {
            break;
        }
    }

    // 释放资源
    cap.release();
    cv::destroyAllWindows();

    return 0;
}



