#include "../common/hik_camera.h"
#include "../common/common.hpp"

HikCamera::HikCamera(){}

HikCamera::~HikCamera(){}

void HikCamera::initialize()
{
    LOG("------------------------ Initialize ------------------------");
    MV3D_RGBD_VERSION_INFO stVersion;
    ASSERT_OK( MV3D_RGBD_GetSDKVersion(&stVersion));
    LOGD("dll version: %d.%d.%d", stVersion.nMajor, stVersion.nMinor, stVersion.nRevision);

    ASSERT_OK(MV3D_RGBD_Initialize());

    unsigned int nDevNum = 0;
    ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum));
    LOGD("MV3D_RGBD_GetDeviceNumber success! nDevNum:%d.", nDevNum);
    ASSERT(nDevNum);

    // 枚举设备信息
    std::vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
    ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum));
    for (unsigned int i = 0; i < nDevNum; i++)
    {  
        if (DeviceType_Ethernet == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] IP[%s] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stNetInfo.chCurrentIp, devs[i].chModelName);
        }
        else if (DeviceType_USB == devs[i].enDeviceType)
        {
            LOG("Index[%d]. SerialNum[%s] UsbProtocol[%d] name[%s].\r\n", i, devs[i].chSerialNumber, devs[i].SpecialInfo.stUsbInfo.enUsbProtocol, devs[i].chModelName);
        }
    }
    LOG("----------------------------------------------------------");
}

void HikCamera::connect(const char* chSerialNumber)
{
    LOG("------------------------ Connect ------------------------");
    if(nullptr == chSerialNumber)
    {
        // 连接第一个设备
        unsigned int nDevNum = 0;
        ASSERT_OK(MV3D_RGBD_GetDeviceNumber(DeviceType_Ethernet | DeviceType_USB, &nDevNum));
        vector<MV3D_RGBD_DEVICE_INFO> devs(nDevNum);
        ASSERT_OK(MV3D_RGBD_GetDeviceList(DeviceType_Ethernet | DeviceType_USB, &devs[0], nDevNum, &nDevNum));
        ASSERT_OK(MV3D_RGBD_OpenDevice(&m_handle, &devs[0]));
        m_strSerialNum = devs[0].chSerialNumber;
        LOGD("Open device success");
    }
    else
    {
        ASSERT_OK(MV3D_RGBD_OpenDeviceBySerialNumber(&m_handle, chSerialNumber));
        m_strSerialNum = chSerialNumber;
        LOGD("Open device by serial number success");
    }
    LOG("----------------------------------------------------------");
}

string HikCamera::getSerialNumber()
{
    if(nullptr == m_handle)
    {
        LOGE("Get serial number failed! No device connected...");
        return nullptr;
    }
    else
    {
        return m_strSerialNum;
    }
}

int HikCamera::setParam(const char* chParam, MV3D_RGBD_PARAM* pstParam)
{
    return MV3D_RGBD_SetParam(m_handle, chParam, pstParam);
}

int HikCamera::getParam(const char* chParam, MV3D_RGBD_PARAM* pstParam)
{
    return MV3D_RGBD_GetParam(m_handle, chParam, pstParam);
}

vector<sensor_msgs::CameraInfo> HikCamera::getCamInfo()
{
    LOG("------------------------ Get cam info ------------------------");
    vector<sensor_msgs::CameraInfo> stCamInfo(2);
    MV3D_RGBD_CAMERA_PARAM stCamParam = {0};
    int nRet = MV3D_RGBD_GetCameraParam(m_handle, &stCamParam);
    if(MV3D_RGBD_OK != nRet)
    {
        LOGE("Get camera param failed...sts[%d]", nRet);
    }
    
    // 相机深度图畸变系数
    vector<double> D0;
    for(double s : stCamParam.stDepthCalibInfo.stDistortion.fData){
        D0.push_back((double)(s));
    }
    // 相机深度图内参
    boost::array<double, 9> K0 = {
        (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[0]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[1]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[2]),
        (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[3]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[4]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[5]),
        (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[6]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[7]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[8]),
    };
    // 深度图坐标系投影矩阵
    boost::array<double, 12> P0 = {
        (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[0]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[1]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[2]), 0.000000,
        (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[3]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[4]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[5]), 0.000000,
        (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[6]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[7]), (double)(stCamParam.stDepthCalibInfo.stIntrinsic.fData[8]), 0.000000
    };
    // 深度图坐标系旋转矩阵
    boost::array<double, 9> R0 = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    stCamInfo[0].width = stCamParam.stDepthCalibInfo.nWidth;
    stCamInfo[0].height = stCamParam.stDepthCalibInfo.nHeight;
    stCamInfo[0].distortion_model = "plumb_bob";
    stCamInfo[0].D = D0;
    stCamInfo[0].K = K0;
    stCamInfo[0].P = P0;
    stCamInfo[0].R = R0;
    stCamInfo[0].binning_x = 0;
    stCamInfo[0].binning_y = 0;
    stCamInfo[0].header.frame_id = "image_frame_id";
    stCamInfo[0].header.stamp = ros::Time::now();
    stCamInfo[0].header.stamp.nsec = 0;

    // 相机彩色图畸变系数
    vector<double> D1;
    for(double s : stCamParam.stRgbCalibInfo.stDistortion.fData){
        D1.push_back((double)(s));
    }
    // 相机彩色图内参
    boost::array<double, 9> K1 = {
        (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[0]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[1]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[2]),
        (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[3]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[4]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[5]),
        (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[6]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[7]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[8]),
    };
    // 彩色图坐标系投影矩阵
    boost::array<double, 12> P1 = {
        (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[0]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[1]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[2]), 0.000000,
        (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[3]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[4]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[5]), 0.000000,
        (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[6]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[7]), (double)(stCamParam.stRgbCalibInfo.stIntrinsic.fData[8]), 0.000000
    };
    // 彩色图坐标系旋转矩阵
    boost::array<double, 9> R1 = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    stCamInfo[1].width = stCamParam.stRgbCalibInfo.nWidth;
    stCamInfo[1].height = stCamParam.stRgbCalibInfo.nHeight;
    stCamInfo[1].distortion_model = "plumb_bob";
    stCamInfo[1].D = D1;
    stCamInfo[1].K = K1;
    stCamInfo[1].P = P1;
    stCamInfo[1].R = R1;
    stCamInfo[1].binning_x = 0;
    stCamInfo[1].binning_y = 0;
    stCamInfo[1].header.frame_id = "image_frame_id";
    stCamInfo[1].header.stamp = ros::Time::now();
    stCamInfo[1].header.stamp.nsec = 0;
    LOG("----------------------------------------------------------");

    return stCamInfo;
}

void HikCamera::start()
{
    LOG("------------------------ Start ------------------------");
    ASSERT_OK(MV3D_RGBD_Start(m_handle));
    LOG("----------------------------------------------------------");
}


void HikCamera::stop()
{
    LOG("------------------------ Stop ------------------------");
    ASSERT_OK(MV3D_RGBD_Stop(m_handle));
    LOG("----------------------------------------------------------");
}

void HikCamera::close()
{
    LOG("------------------------ Close ------------------------");
    ASSERT_OK(MV3D_RGBD_CloseDevice(&m_handle));
    LOG("----------------------------------------------------------");
}

vector<sensor_msgs::ImagePtr> HikCamera::getImage()
{
    m_nMsgImgType = 0;
    vector<sensor_msgs::ImagePtr> stImgPtr(4);
    MV3D_RGBD_FRAME_DATA stFrameData;
    int nRet = MV3D_RGBD_FetchFrame(m_handle, &stFrameData, 5000);
    if(MV3D_RGBD_OK == nRet)
    {
        if(!stFrameData.nValidInfo)
        {
            // 根据图像类型进行赋值
            for(int i = 0; i < stFrameData.nImageCount; i++)
            {
                if(ImageType_Depth == stFrameData.stImageData[i].enImageType)
                {
                    m_matDepthImg = cv::Mat(cv::Size(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight), CV_16UC1, stFrameData.stImageData[i].pData);
                    stImgPtr[0] = cv_bridge::CvImage(std_msgs::Header(), "mono16", m_matDepthImg).toImageMsg();
                    m_nMsgImgType |= TYPE_DEPTH;
                    ROS_INFO("SN:%s Get depth image success", m_strSerialNum.c_str());
                }
                else if(ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType || ImageType_YUV422 == stFrameData.stImageData[i].enImageType)
                {
                    int nDstImageLen = stFrameData.stImageData[i].nWidth * stFrameData.stImageData[i].nHeight * 3;
                    if (nullptr == m_pRgbBuffer || m_nRgbBufferLen < nDstImageLen)
                    {
                        if(nullptr != m_pRgbBuffer)
                        {
                            free(m_pRgbBuffer);
                            m_pRgbBuffer = nullptr;
                        }
                        m_pRgbBuffer = (unsigned char*)malloc(nDstImageLen);
                        if(nullptr == m_pRgbBuffer)
                        {
                            LOGE("SN:%s Convert rgb8planar to rgb_packed failed...", m_strSerialNum.c_str());
                            continue;
                        }
                        memset(m_pRgbBuffer, 0, nDstImageLen);
                        m_nRgbBufferLen = nDstImageLen;
                    }
                    if(ImageType_RGB8_Planar == stFrameData.stImageData[i].enImageType)
                    {
                        RGB8Planar_T_RGB(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].pData, m_pRgbBuffer);
                    }
                    else
                    {
                        YUV422_T_RGB(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight, stFrameData.stImageData[i].pData, m_pRgbBuffer);
                    }
                    m_matRgbImg = cv::Mat(cv::Size(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight), CV_8UC3, m_pRgbBuffer);
                    stImgPtr[1] = cv_bridge::CvImage(std_msgs::Header(), "rgb8", m_matRgbImg).toImageMsg();
                    
                    m_nMsgImgType |= TYPE_RGB;
                    ROS_INFO("SN:%s Get rgb image success", m_strSerialNum.c_str());
                }
                else if(StreamType_Ir_Left == stFrameData.stImageData[i].enStreamType)
                {
                    m_matLeftIrImg = cv::Mat(cv::Size(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight), CV_8UC1, stFrameData.stImageData[i].pData);
                    stImgPtr[2] = cv_bridge::CvImage(std_msgs::Header(), "mono8", m_matLeftIrImg).toImageMsg();
                    m_nMsgImgType |= TYPE_LEFT_IR;
                    ROS_INFO("SN:%s Get left ir image success", m_strSerialNum.c_str());
                }
                else if(StreamType_Ir_Right == stFrameData.stImageData[i].enStreamType)
                {
                    m_matRightIrImg = cv::Mat(cv::Size(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight), CV_8UC1, stFrameData.stImageData[i].pData);
                    stImgPtr[3] = cv_bridge::CvImage(std_msgs::Header(), "mono8", m_matRightIrImg).toImageMsg();
                    m_nMsgImgType |= TYPE_RIGHT_IR;
                    ROS_INFO("SN:%s Get right ir image success", m_strSerialNum.c_str());
                }
            }
            m_nFrameNum = stFrameData.stImageData[0].nFrameNum;
            m_nTimeStamp = stFrameData.stImageData[0].nTimeStamp;
        }
        else
        {
            ROS_INFO("SN:%s Frame is not valid...", m_strSerialNum.c_str());
        }
    }
    else
    {
        ROS_INFO("SN:%s Fetch frame time out...", m_strSerialNum.c_str());
    }
    return stImgPtr;
}

uint32_t HikCamera::getMsgImgType()
{
    return m_nMsgImgType;
}

sensor_msgs::PointCloud2 HikCamera::getPointCloud()
{
    sensor_msgs::PointCloud2 stPointCloud2;
    MV3D_RGBD_FRAME_DATA stFrameData;
    int nRet = MV3D_RGBD_FetchFrame(m_handle, &stFrameData, 5000);
    if(MV3D_RGBD_OK == nRet)
    {
        if(!stFrameData.nValidInfo)
        {
            m_nMsgPointCloudType = 0;
            // 根据图像类型进行赋值
            for(int i = 0; i < stFrameData.nImageCount; i++)
            {
                if (ImageType_PointCloud == stFrameData.stImageData[i].enImageType)
                {
                    // 组包普通点云
                    m_pclPointCloud(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                    convertToPCL(stFrameData.stImageData[i], m_pclPointCloud);
                    pcl::toROSMsg(m_pclPointCloud, stPointCloud2);
                    m_nMsgPointCloudType = TYPE_POINT_CLOUD;
                    ROS_INFO("SN:%s Get point cloud[%d] success", m_strSerialNum.c_str(), stFrameData.stImageData[i].nFrameNum);
                    break;
                }
                else if (ImageType_PointCloudWithNormals == stFrameData.stImageData[i].enImageType)
                {
                    // 组包法向量点云
                    m_pclPointCloudWithNormals(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                    convertToPCL(stFrameData.stImageData[i], m_pclPointCloudWithNormals);
                    pcl::toROSMsg(m_pclPointCloudWithNormals, stPointCloud2);
                    m_nMsgPointCloudType = TYPE_POINT_CLOUD_WITH_NORMALS;
                    ROS_INFO("SN:%s Get point cloud with normals[%d] success", m_strSerialNum.c_str(), stFrameData.stImageData[i].nFrameNum);
                    break;
                }
                else if (ImageType_TexturedPointCloud == stFrameData.stImageData[i].enImageType)
                {
                    // 组包纹理点云
                    m_pclTexturedPointCloud(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                    convertToPCL(stFrameData.stImageData[i], m_pclTexturedPointCloud);
                    pcl::toROSMsg(m_pclTexturedPointCloud, stPointCloud2);
                    m_nMsgPointCloudType = TYPE_TEXTURED_POINT_CLOUD;
                    ROS_INFO("SN:%s Get textured point cloud[%d] success", m_strSerialNum.c_str(), stFrameData.stImageData[i].nFrameNum);
                    break;
                }
                else if (ImageType_TexturedPointCloudWithNormals == stFrameData.stImageData[i].enImageType)
                {
                    // 组包带法向量的纹理点云
                    m_pclTexturedPointCloudWithNormals(stFrameData.stImageData[i].nWidth, stFrameData.stImageData[i].nHeight);
                    convertToPCL(stFrameData.stImageData[i], m_pclTexturedPointCloudWithNormals);
                    pcl::toROSMsg(m_pclTexturedPointCloudWithNormals, stPointCloud2);
                    m_nMsgPointCloudType = TYPE_TEXTURED_POINT_CLOUD_WITH_NORMALS;
                    ROS_INFO("SN:%s Get textured point cloud with normals[%d] success", m_strSerialNum.c_str(), stFrameData.stImageData[i].nFrameNum);
                    break;
                }
            }     
            m_nFrameNum = stFrameData.stImageData[0].nFrameNum;
            m_nTimeStamp = stFrameData.stImageData[0].nTimeStamp;
        }
        else
        {
            ROS_INFO("Frame is not valid...");
        }
    }
    else
    {
        ROS_INFO("Fetch frame time out...");
    }
    return stPointCloud2;
}

// 获取帧号
uint32_t HikCamera::getFrameNum()
{
    return m_nFrameNum;
}

// 获取时戳
int64_t HikCamera::getTimeStamp()
{
    return m_nTimeStamp;
}

// 普通点云转换
void HikCamera::convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointXYZ>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = stPointCloud.nHeight;
    pclCloud2.width = stPointCloud.nWidth;
    pclCloud2.point_step = sizeof(POINT_XYZ);
    pclCloud2.row_step = sizeof(POINT_XYZ) * stPointCloud.nWidth;

    pclCloud2.fields.reserve(3);
    pclCloud2.fields.push_back(createPointField("x", offsetof(POINT_XYZ, fX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(POINT_XYZ, fY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("z", offsetof(POINT_XYZ, fZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * stPointCloud.nHeight);
    memcpy(pclCloud2.data.data(), stPointCloud.pData, stPointCloud.nDataLen);
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
    return;
}

// 法向量点云转换
void HikCamera::convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointNormal>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = stPointCloud.nHeight;
    pclCloud2.width = stPointCloud.nWidth;
    pclCloud2.point_step = sizeof(POINT_XYZNORMALS);
    pclCloud2.row_step = sizeof(POINT_XYZNORMALS) * stPointCloud.nWidth;

    pclCloud2.fields.reserve(7);
    pclCloud2.fields.push_back(createPointField("x", offsetof(POINT_XYZNORMALS, pPoint.fX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(POINT_XYZNORMALS, pPoint.fY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("z", offsetof(POINT_XYZNORMALS, pPoint.fZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_x", offsetof(POINT_XYZNORMALS, pNormals.fNormalX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_y", offsetof(POINT_XYZNORMALS, pNormals.fNormalY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_z", offsetof(POINT_XYZNORMALS, pNormals.fNormalZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("curvature", offsetof(POINT_XYZNORMALS, pNormals.fCurvature), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * stPointCloud.nHeight);
	uint8_t* pData = pclCloud2.data.data();
	uint8_t* pPointCloudData = stPointCloud.pData;
	for (int i = 0; i < stPointCloud.nHeight * stPointCloud.nWidth; i++)
	{
		memcpy(pData, pPointCloudData, sizeof(MV3D_RGBD_POINT_XYZ_NORMALS));
		pData += pclCloud2.point_step;
		pPointCloudData += sizeof(MV3D_RGBD_POINT_XYZ_NORMALS);
	}
	pData = NULL;
	pPointCloudData = NULL;
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
    return;
}

// 纹理点云转换
void HikCamera::convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointXYZRGB>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = stPointCloud.nHeight;
    pclCloud2.width = stPointCloud.nWidth;
    pclCloud2.point_step = sizeof(POINT_XYZBGR);
    pclCloud2.row_step = sizeof(POINT_XYZBGR) * stPointCloud.nWidth;

    pclCloud2.fields.reserve(4);
    pclCloud2.fields.push_back(createPointField("x", offsetof(POINT_XYZBGR, fX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(POINT_XYZBGR, fY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("z", offsetof(POINT_XYZBGR, fZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("rgb", offsetof(POINT_XYZBGR, fRgb), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * stPointCloud.nHeight);
	memcpy(pclCloud2.data.data(), stPointCloud.pData, stPointCloud.nDataLen);
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
	return;
}

// 带法向量的纹理点云转换
void HikCamera::convertToPCL(const MV3D_RGBD_IMAGE_DATA& stPointCloud, pcl::PointCloud<pcl::PointXYZRGBNormal>& pclPointCloud)
{
    pcl::PCLPointCloud2 pclCloud2;
    pclCloud2.height = stPointCloud.nHeight;
    pclCloud2.width = stPointCloud.nWidth;
    pclCloud2.point_step = sizeof(POINT_XYZRGBNORMALS);
    pclCloud2.row_step = sizeof(POINT_XYZRGBNORMALS) * stPointCloud.nWidth;

    pclCloud2.fields.reserve(8);
    pclCloud2.fields.push_back(createPointField("x", offsetof(POINT_XYZRGBNORMALS, pColorPoint.fX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
    pclCloud2.fields.push_back(createPointField("y", offsetof(POINT_XYZRGBNORMALS, pColorPoint.fY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("z", offsetof(POINT_XYZRGBNORMALS, pColorPoint.fZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("rgb", offsetof(POINT_XYZRGBNORMALS, pColorPoint.fRgb), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_x", offsetof(POINT_XYZRGBNORMALS, pNormals.fNormalX), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_y", offsetof(POINT_XYZRGBNORMALS, pNormals.fNormalY), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("normal_z", offsetof(POINT_XYZRGBNORMALS, pNormals.fNormalZ), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));
	pclCloud2.fields.push_back(createPointField("curvature", offsetof(POINT_XYZRGBNORMALS, pNormals.fCurvature), pcl::PCLPointField::PointFieldTypes::FLOAT32, 1));

    pclCloud2.data.resize(pclCloud2.row_step * stPointCloud.nHeight);
	uint8_t* pData = pclCloud2.data.data();
	uint8_t* pPointCloudData = stPointCloud.pData;
	for (int i = 0; i < stPointCloud.nHeight * stPointCloud.nWidth; i++)
	{
		memcpy(pData, pPointCloudData, sizeof(MV3D_RGBD_POINT_XYZ_RGB_NORMALS));
		pData += sizeof(POINT_XYZRGBNORMALS);
		pPointCloudData += sizeof(MV3D_RGBD_POINT_XYZ_RGB_NORMALS);
	}
    pcl::fromPCLPointCloud2(pclCloud2, pclPointCloud);
    return;
}

// 通过OpenCV保存图像
bool HikCamera::saveImage(const char* chFileName, int enImageMsgType)
{
    if(m_nMsgImgType & enImageMsgType)
    {
        switch(enImageMsgType)
        {
        case TYPE_DEPTH:
            cv::imwrite(chFileName, m_matDepthImg);
        break;
        case TYPE_RGB:
            cv::imwrite(chFileName, m_matRgbImg);
        break;
        case TYPE_LEFT_IR:
            cv::imwrite(chFileName, m_matLeftIrImg);
        break;
        case TYPE_RIGHT_IR:
            cv::imwrite(chFileName, m_matRightIrImg);
        break;
        default:
        break;
        }
    }
    else
    {
        ROS_INFO("SN:%s Save image type[%d] failed...", m_strSerialNum, enImageMsgType);
        return false;
    }
    return true;
}

// 通过PCL保存点云
bool HikCamera::savePointCloud(const char* chFileName, int enPointCloudMsgType)
{
    if(m_nMsgPointCloudType == enPointCloudMsgType)
    {
        pcl::PCDWriter pcdWriter; 
        switch(enPointCloudMsgType)
        {
        case TYPE_POINT_CLOUD:
            pcdWriter.write(chFileName, m_pclPointCloud, false);
        break;
        case TYPE_TEXTURED_POINT_CLOUD:
            pcdWriter.write(chFileName, m_pclTexturedPointCloud, false);
        break;
        case TYPE_POINT_CLOUD_WITH_NORMALS:
            pcdWriter.write(chFileName, m_pclPointCloudWithNormals, false);
        break;
        case TYPE_TEXTURED_POINT_CLOUD_WITH_NORMALS:
            pcdWriter.write(chFileName, m_pclTexturedPointCloudWithNormals, false);
        break;
        default:
        break;
        }
    }
    else
    {
        ROS_INFO("SN:%s Save point cloud type[%d] failed...", m_strSerialNum, enPointCloudMsgType);
        return false;
    }
    return true;
}

// 获取SDK得到的点云类型
uint32_t HikCamera::getMsgPointCloudType()
{
    return m_nMsgPointCloudType;
}