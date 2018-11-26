// Copyright (c) 2018 JachinShen(jachinshen@foxmail.com)
//
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "camera/camera_wrapper.h"

using std::cout;
using std::endl;

int GlobalShutterCamera::init() {
  iCameraCounts = 1;
  iStatus = -1;
  iplImage = NULL;
  channel = 3;

  CameraSdkInit(1);

  //枚举设备，并建立设备列表
  CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);

  //没有连接设备
  if (iCameraCounts == 0) {
    return -1;
  }

  //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
  iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

  //初始化失败
  if (iStatus != CAMERA_STATUS_SUCCESS) {
    return -1;
  }

  //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
  CameraGetCapability(hCamera, &tCapability);

  // set resolution to 320*240
  // CameraSetImageResolution(hCamera, &(tCapability.pImageSizeDesc[2]));
  g_pRgbBuffer =
      (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax *
                             tCapability.sResolutionRange.iWidthMax * 3);
  // g_readBuf = (unsigned
  // char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
  CameraSetAeState(hCamera, true);
  // CameraSetExposureTime(hCamera, 6000);
  // CameraSetAeTarget(hCamera, -10000);
  // int ae_target;
  // CameraGetAeTarget(hCamera, &ae_target);
  // cout << "Ae Target: " << ae_target << endl;
  /*让SDK进入工作模式，开始接收来自相机发送的图像
  数据。如果当前相机是触发模式，则需要接收到
  触发帧以后才会更新图像。    */
  CameraPlay(hCamera);

  /*其他的相机参数设置
  例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
       CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
       CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
       更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
  */

  if (tCapability.sIspCapacity.bMonoSensor) {
    channel = 1;
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
  } else {
    channel = 3;
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
  }
  return 0;
}

bool GlobalShutterCamera::read(cv::Mat& src) {
#if BAYER_HACKING == HACKING_ON
  return read_raw(src);
#elif BAYER_HACKING == HACKING_OFF
  return read_processed(src);
#endif
}

bool GlobalShutterCamera::read_processed(cv::Mat& src) {
  if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) ==
      CAMERA_STATUS_SUCCESS) {
    // double exposure;
    // CameraGetExposureTime(hCamera, &exposure);
    // cout << "Exposure Time: " << exposure << endl;
    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
    if (iplImage) {
      cvReleaseImageHeader(&iplImage);
    }
    iplImage = cvCreateImageHeader(
        cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, channel);
    cvSetData(
        iplImage, g_pRgbBuffer,
        sFrameInfo.iWidth *
            channel);  //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
    src = cv::cvarrToMat(iplImage);

    //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
    //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
    return true;
  } else {
    return false;
  }
}

bool GlobalShutterCamera::read_raw(cv::Mat& src) {
  if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) ==
      CAMERA_STATUS_SUCCESS) {
    if (iplImage) {
      cvReleaseImageHeader(&iplImage);
    }
    iplImage = cvCreateImageHeader(
        cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight), IPL_DEPTH_8U, 1);
    cvSetData(
        iplImage, pbyBuffer,
        sFrameInfo
            .iWidth);  //此处只是设置指针，无图像块数据拷贝，不需担心转换效率
    src = cv::cvarrToMat(iplImage);

    //在成功调用CameraGetImageBuffer后，必须调用CameraReleaseImageBuffer来释放获得的buffer。
    //否则再次调用CameraGetImageBuffer时，程序将被挂起一直阻塞，直到其他线程中调用CameraReleaseImageBuffer来释放了buffer
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
    return true;
  } else {
    return false;
  }
}

GlobalShutterCamera::~GlobalShutterCamera() {
  CameraUnInit(hCamera);
  //注意，现反初始化后再free
  free(g_pRgbBuffer);
}