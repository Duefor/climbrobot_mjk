import threading 
import ctypes 
import time 
import os 
import sys
import rospy
from sensor_msgs.msg import Image
try:
    import numpy as np
    import cv2
    CV2_AVAILABLE = True
except Exception:
    CV2_AVAILABLE = False
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)


from Mv3dRgbdImport.Mv3dRgbdApi import * 
from Mv3dRgbdImport.Mv3dRgbdDefine import (
    DeviceType_Ethernet,
    DeviceType_USB,
    MV3D_RGBD_FLOAT_EXPOSURETIME,
    ParamType_Enum,
    ParamType_Int,
    ParamType_Float,
    ParamType_String,
    ParamType_Bool,
    MV3D_RGBD_FLOAT_Z_UNIT,
    MV3D_RGBD_DEVICE_INFO_LIST,
    MV3D_RGBD_FRAME_DATA,
    MV3D_RGBD_PARAM,
    MV3D_RGBD_ENUM_WORKINGMODE,
    MV3D_RGBD_INT_IMAGEALIGN,
    MV3D_RGBD_ENUM_RESOLUTION,
    MV3D_RGBD_ENUM_IMAGEMODE,
    MV3D_RGBD_OK,
    ImageType_Depth,
    ImageType_Rgbd,
    ImageType_YUV422,
    ImageType_RGB8_Planar,
    ImageType_YUV420SP_NV21,
    ImageType_YUV420SP_NV12,
    ImageType_Jpeg,
    ImageType_Mono8,
)



if __name__ == "__main__": 
    # 创建一个c语言无符号整型变量
    nDeviceNum=ctypes.c_uint(0)
    nDeviceNum_p=byref(nDeviceNum)
    # |or运算符
    ret=Mv3dRgbd.MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | 
DeviceType_USB, nDeviceNum_p) #获取设备数量 
    if ret!=0:
        print("MV3D_RGBD_GetDeviceNumber fail! ret[0x%x]" % ret)
        input('Press Enter to continue...')
        sys.exit()
    if nDeviceNum.value==0:
        print("find no device!")
        input('Press Enter to continue...')
        sys.exit()
    print("Find devices numbers:", nDeviceNum.value) 
 
    stDeviceList = MV3D_RGBD_DEVICE_INFO_LIST()
    net = Mv3dRgbd.MV3D_RGBD_GetDeviceList(
        DeviceType_Ethernet | DeviceType_USB,
        pointer(stDeviceList.DeviceInfo[0]),
        20,
        nDeviceNum_p,
    )
    for i in range(0, nDeviceNum.value): 
        print("\ndevice: [%d]" % i) 
        strModeName = "" 
        for per in stDeviceList.DeviceInfo[i].chModelName: 
            strModeName = strModeName + chr(per) 
        print("device model name: %s" % strModeName) 
 
        strSerialNumber = "" 
        for per in stDeviceList.DeviceInfo[i].chSerialNumber: 
            strSerialNumber = strSerialNumber + chr(per) 
        print("device SerialNumber: %s" % strSerialNumber) 
 
    # 创建相机示例 
    camera=Mv3dRgbd()
    nConnectionNum = input("please input the number of the device to connect:")
    if int(nConnectionNum) >= nDeviceNum.value:
        print("intput error!")
        input('Press Enter to continue...')
        sys.exit()

    # 打开设备
    ret=camera.MV3D_RGBD_OpenDevice(pointer(stDeviceList.DeviceInfo[int(nConnectionNum)]))
    if ret!=0:
        print ("MV3D_RGBD_OpenDevice fail! ret[0x%x]" % ret)
        input('Press Enter to continue...')
        sys.exit()

    # ---------------------------
    # 设置相机参数：工作模式、深度图对齐、采集分辨率(binning)
    # ---------------------------
    # 设置 CameraWorkingMode（枚举） — 深度图模式，可选重复设置，OpenDevice 可能已切换
    try:
        p = MV3D_RGBD_PARAM()
        p.enParamType = ParamType_Enum
        p.ParamInfo.stEnumParam.nCurValue = 4
        ret = camera.MV3D_RGBD_SetParam(MV3D_RGBD_ENUM_WORKINGMODE, pointer(p))
        if ret != MV3D_RGBD_OK:
            print("Set CameraWorkingMode failed: 0x%x" % ret)
    except Exception as e:
        print("Setting CameraWorkingMode exception:", e)

    # 设置 ImageAlign 为 1（深度图对齐）
    try:
        p = MV3D_RGBD_PARAM()
        p.enParamType = ParamType_Int
        p.ParamInfo.stIntParam.nCurValue = 1
        ret = camera.MV3D_RGBD_SetParam(MV3D_RGBD_INT_IMAGEALIGN, pointer(p))
        if ret != MV3D_RGBD_OK:
            print("Set ImageAlign failed: 0x%x" % ret)
    except Exception as e:
        print("Setting ImageAlign exception:", e)

    # 关闭 binning（使用你提供的值 0x00010001 表示关闭）
    try:
        p = MV3D_RGBD_PARAM()
        p.enParamType = ParamType_Enum
        p.ParamInfo.stEnumParam.nCurValue = 0x00010001
        ret = camera.MV3D_RGBD_SetParam(MV3D_RGBD_ENUM_RESOLUTION, pointer(p))
        if ret != MV3D_RGBD_OK:
            print("Set Binning/Resolution failed: 0x%x" % ret)
    except Exception as e:
        print("Setting Binning exception:", e)

    # 初始化 ROS publisher（如果可用）
    ROS_AVAILABLE = True
    try:
        if not rospy.core.is_initialized():
            rospy.init_node('mv3d_camera_node', anonymous=True)
        rgb_image_pub = rospy.Publisher('/camera/rgb', Image, queue_size=10)
        depth_image_pub = rospy.Publisher('/camera/depth', Image, queue_size=10)

    except Exception as e:
        print('Failed to init ROS publisher:', e)
        ROS_AVAILABLE = False

    # 开始取流 
    ret=camera.MV3D_RGBD_Start()
    if ret != 0:
        print ("start fail! ret[0x%x]" % ret)
        camera.MV3D_RGBD_CloseDevice()
        input('Press Enter to continue...')
        sys.exit()
 
    # time_start=time.time()
    try:
        while not rospy.is_shutdown():
            stFrameData=MV3D_RGBD_FRAME_DATA()
            ret=camera.MV3D_RGBD_FetchFrame(pointer(stFrameData), 5000)
            if ret==0:
                for i in range(0, stFrameData.nImageCount):
                    img_info = stFrameData.stImageData[i]
                    print(
                        "MV3D_RGBD_FetchFrame[%d]:enImageType[%d],nWidth[%d],nHeight[%d],"
                        ",nDataLen[%d],nFrameNum[%d],bIsRectified[%d],enStreamType[%d],"
                        "enCoordinateType[%d]" % (
                            i,
                            img_info.enImageType,
                            img_info.nWidth,
                            img_info.nHeight,
                            img_info.nDataLen,
                            img_info.nFrameNum,
                            img_info.bIsRectified,
                            img_info.enStreamType,
                            img_info.enCoordinateType,
                        )
                    )

                    # Prepare raw bytes from C pointer
                    try:
                        data_bytes = ctypes.string_at(img_info.pData, img_info.nDataLen)
                    except Exception:
                        data_bytes = None

                    # Publish depth image
                    if ROS_AVAILABLE and data_bytes is not None:
                        try:
                            etype = img_info.enImageType
                            width = img_info.nWidth
                            height = img_info.nHeight
                            if etype == ImageType_Depth:
                                msg = Image()
                                msg.header.stamp = rospy.Time.now()
                                msg.height = height
                                msg.width = width
                                msg.encoding = '16UC1'
                                msg.is_bigendian = 0
                                msg.step = width * 2
                                msg.data = data_bytes
                                depth_image_pub.publish(msg)

                            # Color/JPEG/other -> try to publish as rgb8 or mono8
                            elif etype == ImageType_YUV422:
                                print("JPEG image received")
                                if CV2_AVAILABLE:
                                    arr = np.frombuffer(data_bytes, dtype=np.uint8)
                                    bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                                    if bgr is not None:
                                        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                                        msg = Image()
                                        msg.header.stamp = rospy.Time.now()
                                        msg.height = rgb.shape[0]
                                        msg.width = rgb.shape[1]
                                        msg.encoding = 'rgb8'
                                        msg.is_bigendian = 0
                                        msg.step = msg.width * 3
                                        msg.data = rgb.tobytes()
                                        rgb_image_pub.publish(msg)
                            else:
                                # assume raw RGB or mono data
                                # try mono8
                                if img_info.nDataLen == width * height:
                                    msg = Image()
                                    msg.header.stamp = rospy.Time.now()
                                    msg.height = height
                                    msg.width = width
                                    msg.encoding = 'mono8'
                                    msg.is_bigendian = 0
                                    msg.step = width
                                    msg.data = data_bytes
                                    rgb_image_pub.publish(msg)
                                elif img_info.nDataLen == width * height * 3:
                                    msg = Image()
                                    msg.header.stamp = rospy.Time.now()
                                    msg.height = height
                                    msg.width = width
                                    msg.encoding = 'rgb8'
                                    msg.is_bigendian = 0
                                    msg.step = width * 3
                                    msg.data = data_bytes
                                    rgb_image_pub.publish(msg)
                        except Exception as e:
                            print('Publish image exception:', e)
            else:
                print("no data[0x%x]" % ret)
            # time_end=time.time()
            # sum_t=time_end - time_start
            # # 取流超过10s后退出
            # if sum_t>10:
            #     break
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, exiting loop')

    # 停止取流 
    ret=camera.MV3D_RGBD_Stop() 
    if ret != 0: 
        print ("stop fail! ret[0x%x]" % ret) 
        input('Press Enter to continue...') 
        sys.exit() 

    # 销毁句柄 
    ret=camera.MV3D_RGBD_CloseDevice() 
    if ret != 0: 
        print ("CloseDevice fail! ret[0x%x]" % ret) 
        input('Press Enter to continue...') 
        sys.exit() 
    
    sys.exit()