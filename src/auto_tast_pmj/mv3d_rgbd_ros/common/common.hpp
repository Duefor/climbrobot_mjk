#ifndef SAMPLE_COMMON_COMMON_HPP_
#define SAMPLE_COMMON_COMMON_HPP_

#include "Utils.hpp"
#include "ConvertImageType.hpp"

#include <fstream>
#include <iterator>
#include <sys/stat.h>
#include <sys/types.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>

typedef struct _POINT_XYZ_
{
	float fX;
	float fY;
	float fZ;
}POINT_XYZ;

typedef struct _POINT_NORMAL_
{
	float fNormalX;
	float fNormalY;
	float fNormalZ;
	float fCurvature;
}POINT_NORMAL;

typedef struct _POINT_XYZBGR_
{
	float fX;
	float fY;
	float fZ;

	union {
		struct
		{
			uint8_t nB;
			uint8_t nG;
			uint8_t nR;
			uint8_t nA;
		};
		float fRgb;
	};
}POINT_XYZBGR;

typedef struct _POINT_XYZNORMALS_
{
	POINT_XYZ pPoint;
	POINT_NORMAL pNormals;
}POINT_XYZNORMALS;

typedef struct _POINT_XYZRGBNORMALS_
{
	POINT_XYZBGR pColorPoint;
	POINT_NORMAL pNormals;
}POINT_XYZRGBNORMALS;

typedef struct _RGB_TEXTURE_DATA_
{
	uint8_t nR;
	uint8_t nG;
	uint8_t nB;
}RGB_TEXTURE_DATA;

pcl::PCLPointField createPointField(std::string strName, uint32_t nOffset, uint8_t nDataType, uint32_t nCount)
{
	pcl::PCLPointField pclField;
	pclField.name = strName;
	pclField.offset = nOffset;
	pclField.datatype = nDataType;
	pclField.count = nCount;
	return pclField;
}

#endif
