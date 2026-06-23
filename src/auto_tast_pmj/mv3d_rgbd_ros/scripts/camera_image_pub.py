#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os
import time
import ctypes
from ctypes import byref, pointer
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge
import cv2
import copy


current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

from Mv3dRgbdImport.Mv3dRgbdApi import *
from Mv3dRgbdImport.Mv3dRgbdDefine import (
    DeviceType_Ethernet,
    DeviceType_USB,
    MV3D_RGBD_PARAM,
    ParamType_Enum,
    ParamType_Int,
    MV3D_RGBD_ENUM_WORKINGMODE,
    MV3D_RGBD_INT_IMAGEALIGN,
    MV3D_RGBD_ENUM_RESOLUTION,
    MV3D_RGBD_OK,
    MV3D_RGBD_DEVICE_INFO_LIST,
    MV3D_RGBD_FRAME_DATA,
    ImageType_Depth,
    ImageType_Jpeg,
    ImageType_YUV422,
    MV3D_RGBD_CALIB_INFO,
    MV3D_RGBD_CAMERA_PARAM,
    CoordinateType_RGB,
    CoordinateType_Depth
)



def build_camera_info_from_calib(calib):
    # calib: MV3D_RGBD_CALIB_INFO
    ci = CameraInfo()
    width = int(calib.nWidth)
    height = int(calib.nHeight)
    ci.width = width
    ci.height = height
    # Intrinsic matrix is stored row-major in fData (9 elements)
    K = list(calib.stIntrinsic.fData)
    ci.K = [float(x) for x in K]
    # Projection matrix P (3x4): [fx 0 cx 0; 0 fy cy 0; 0 0 1 0]
    fx = float(K[0])
    fy = float(K[4])
    cx = float(K[2])
    cy = float(K[5])
    ci.P = [fx, 0.0, cx, 0.0,
        0.0, fy, cy, 0.0,
        0.0, 0.0, 1.0, 0.0]
    # Distortion coefficients: SDK stores 12 floats: k1,k2,p1,p2,k3,k4,k5,k6,s1,s2,s3,s4
    dist = list(calib.stDistortion.fData)
    # sensor_msgs uses D array; choose first 5 coefficients (plumb_bob): k1,k2,p1,p2,k3
    ci.distortion_model = 'plumb_bob'
    ci.D = [float(dist[0]), float(dist[1]), float(dist[2]), float(dist[3]), float(dist[4])]
    # Rectification matrix R (identity unless stereo rectification applied)
    ci.R = [1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0]
    return ci


def main():
    rospy.init_node('mv3d_image_publisher', anonymous=True)
    rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=5)
    depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=5)
    rgb_info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)
    depth_info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)


    bridge = CvBridge()

    # enumerate devices
    nDeviceNum = ctypes.c_uint(0)
    nDeviceNum_p = byref(nDeviceNum)
    ret = Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, nDeviceNum_p)
    if ret != 0 or nDeviceNum.value == 0:
        rospy.logerr('No devices found or error (%s)' % hex(ret))
        return

    stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
    Mv3dRgbd.MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, pointer(stDeviceList.DeviceInfo[0]), 20, nDeviceNum_p)

    cam = Mv3dRgbd()
    # open first device
    ret = cam.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[0]))
    if ret != 0:
        rospy.logerr('MV3D_RGBD_OpenDevice fail! ret[0x%x]' % ret)
        return

    #设置工作模式 = RGBD模式 (4)
    p = MV3D_RGBD_PARAM()
    p.enParamType = ParamType_Enum
    p.ParamInfo.stEnumParam.nCurValue = 4
    ret = cam.MV3D_RGBD_SetParam(MV3D_RGBD_ENUM_WORKINGMODE, pointer(p))
    if ret != MV3D_RGBD_OK:
        rospy.logwarn('SetParam WORKINGMODE failed: 0x%x' % ret)

    # 设置图像对齐
    p = MV3D_RGBD_PARAM()
    p.enParamType = ParamType_Int
    p.ParamInfo.stIntParam.nCurValue = 1
    ret = cam.MV3D_RGBD_SetParam(MV3D_RGBD_INT_IMAGEALIGN, pointer(p))
    if ret != MV3D_RGBD_OK:
        rospy.logwarn('SetParam IMAGEALIGN failed: 0x%x' % ret)

    # 禁用 Binning (像素合并) (0x00010001)
    p = MV3D_RGBD_PARAM()
    p.enParamType = ParamType_Enum
    p.ParamInfo.stEnumParam.nCurValue = 0x00010001
    ret = cam.MV3D_RGBD_SetParam(MV3D_RGBD_ENUM_RESOLUTION, pointer(p))
    if ret != MV3D_RGBD_OK:
        rospy.logwarn('SetParam RESOLUTION failed: 0x%x' % ret)


    # start
    ret = cam.MV3D_RGBD_Start()
    if ret != 0:
        rospy.logerr('MV3D_RGBD_Start fail! ret[0x%x]' % ret)
        cam.MV3D_RGBD_CloseDevice()
        return

    rospy.loginfo('Started streaming from MV3D camera')
    # get calibration info
    rgb_calib = MV3D_RGBD_CALIB_INFO()
    depth_calib = MV3D_RGBD_CALIB_INFO()
    ret_rgb = cam.MV3D_RGBD_GetCalibInfo(CoordinateType_RGB, pointer(rgb_calib))
    ret_depth = cam.MV3D_RGBD_GetCalibInfo(CoordinateType_Depth, pointer(depth_calib))
    if ret_rgb != MV3D_RGBD_OK or ret_depth != MV3D_RGBD_OK:
        rospy.logerr('MV3D_RGBD_GetCalibInfo failed: rgb=0x%x depth=0x%x' % (ret_rgb, ret_depth))
        cam.MV3D_RGBD_Stop()
        cam.MV3D_RGBD_CloseDevice()
        return

    camera_info_rgb = build_camera_info_from_calib(rgb_calib)
    camera_info_depth = build_camera_info_from_calib(depth_calib)

    # fetch loop
    timeout_ms = 1000

    base_frame = 'base_link' 
    camera_frame = 'camera_link'

    try:
        while not rospy.is_shutdown():
            stFrameData = MV3D_RGBD_FRAME_DATA()
            # 这里FetchFrame保证了物理上的同一帧，
            #stFrameData 里面有一个数组 stImageData，它的长度 nImageCount 通常是 2。
            ret = cam.MV3D_RGBD_FetchFrame(pointer(stFrameData), timeout_ms)
            if ret != MV3D_RGBD_OK:
                continue
            
            current_frame_time = rospy.Time.now()

            #for不是在时间线上遍历，而是在同一帧下去遍历，第一轮i=0是深度图，第二轮i=1是对应帧的RGB图
            for i in range(0, stFrameData.nImageCount):
                img = stFrameData.stImageData[i]
                if not img.pData or img.nDataLen == 0:
                    continue
                data = ctypes.string_at(img.pData, img.nDataLen)

                width = img.nWidth
                height = img.nHeight
                frame_id = 'camera_link'

                if img.enImageType == ImageType_Depth:
                    arr = np.frombuffer(data, dtype=np.uint16)
                    if arr.size != width * height:
                        continue
                    arr = arr.reshape((height, width))
                    ros_img = bridge.cv2_to_imgmsg(arr, encoding='16UC1')
                    ros_img.header.stamp = current_frame_time
                    ros_img.header.frame_id = frame_id
                    depth_pub.publish(ros_img)
                    ci = copy.deepcopy(camera_info_depth)
                    ci.header.stamp = current_frame_time
                    ci.header.frame_id = frame_id
                    depth_info_pub.publish(ci)

                elif img.enImageType == ImageType_YUV422:
                    if img.nDataLen != width * height * 2:
                        continue
                    yuv = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 2))
                    try:
                        rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_YUY2)
                        # rospy.loginfo("Converted YUV to RGB using YUY2")
                    except Exception:
                        try:
                            rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_UYVY)
                        except Exception:
                            continue
                    ros_img = bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
                    ros_img.header.stamp = current_frame_time   
                    ros_img.header.frame_id = frame_id
                    rgb_pub.publish(ros_img)
                    ci = copy.deepcopy(camera_info_rgb)
                    ci.header.stamp = current_frame_time
                    ci.header.frame_id = frame_id
                    rgb_info_pub.publish(ci)
                # else:
                #     if img.nDataLen == width * height * 3:
                #         arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width, 3))
                #         ros_img = bridge.cv2_to_imgmsg(arr, encoding='rgb8')
                #         ros_img.header.stamp = rospy.Time.now()
                #         ros_img.header.frame_id = frame_id
                #         rgb_pub.publish(ros_img)
                #         ci = copy.deepcopy(camera_info_rgb)
                #         ci.header.stamp = ros_img.header.stamp
                #         ci.header.frame_id = frame_id
                #         rgb_info_pub.publish(ci)
                #     elif img.nDataLen == width * height:
                #         arr = np.frombuffer(data, dtype=np.uint8).reshape((height, width))
                #         ros_img = bridge.cv2_to_imgmsg(arr, encoding='mono8')
                #         ros_img.header.stamp = rospy.Time.now()
                #         ros_img.header.frame_id = frame_id
                #         rgb_pub.publish(ros_img)
                #         ci = copy.deepcopy(camera_info_rgb)
                #         ci.header.stamp = ros_img.header.stamp
                #         ci.header.frame_id = frame_id
                #         rgb_info_pub.publish(ci)
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
    finally:
        cam.MV3D_RGBD_Stop()
        cam.MV3D_RGBD_CloseDevice()



if __name__ == '__main__':
    main()

