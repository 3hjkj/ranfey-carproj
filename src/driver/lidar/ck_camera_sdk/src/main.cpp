#include <iostream>
#include <opencv2/opencv.hpp>
#include "CKCameraInterface.h"                     // 相机 SDK 头

#include "rclcpp/rclcpp.hpp"                       // ROS2
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  /* ——— ROS2 初始化 ——— */
  rclcpp::init(argc, argv);
  /* NodeHandle 在 ROS2 中用 rclcpp::Node 替代 */
  auto node = rclcpp::Node::make_shared("camera_node");
  image_transport::ImageTransport it(node);
  auto pub = it.advertise("camera/image", 1);

  /* ——— 相机 SDK 初始化 ——— */
  HANDLE hCamera = nullptr;
  int cam_cnt = 0;
  if (CameraEnumerateDevice(&cam_cnt) != CAMERA_STATUS_SUCCESS || cam_cnt == 0) {
    RCLCPP_ERROR(node->get_logger(), "No camera found!");
    return -1;
  }
  if (CameraInit(&hCamera, 0) != CAMERA_STATUS_SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Camera init failed!");
    return -1;
  }

  CameraReadParameterFromFile(
    hCamera,
    "/home/nvidia/zhitai/qingling_ros2/src/driver/lidar/ck_camera_sdk/src/UGSMT200C_Cfg_A.bin");
  CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
  CameraPlay(hCamera);

  RCLCPP_INFO(node->get_logger(),
              "CKCamera started — press Ctrl‑C in terminal to quit");

  /* ——— 主循环 ——— */
  while (rclcpp::ok()) {
    stImageInfo info;
    BYTE *pImage = CameraGetImageBufferEx(hCamera, &info, 100);   // 100 ms 超时
    if (pImage) {
      Mat frame(info.iHeight, info.iWidth, CV_8UC3, pImage);

      /* 转成 ROS2 Image 消息并发布 */
      auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      resize(frame, frame, Size(1280, 720));
      imshow("CKCamera Display", frame);
       
    }
    // int key = waitKey(30);
    // if (key == 27) break;
    //取消显示窗口

    rclcpp::spin_some(node);   // 处理潜在回调（这里主要是让 Ctrl‑C 生效）
  }

  /* ——— 清理 ——— */
  CameraPause(hCamera);
  CameraUnInit(hCamera);
  destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}
