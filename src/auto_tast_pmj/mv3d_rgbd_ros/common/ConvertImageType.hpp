#ifndef CONVERT_IMAGE_TYPE_HPP
#define CONVERT_IMAGE_TYPE_HPP

#include <stdio.h>
#include <stdlib.h>

static int CoverYuv422_T_RGB_Pixel(int y, int u, int v)
{
    unsigned int pixel32 = 0;
    unsigned char *pixel = (unsigned char *)&pixel32;
    int r, g, b;
    r = y + (1.370705 * (v - 128));
    g = y - (0.698001 * (v - 128)) - (0.337633 * (u - 128));
    b = y + (1.732446 * (u - 128));
    if (r > 255)
    {
        r = 255;
    }
    if (g > 255)
    {
        g = 255;
    }
    if (b > 255)
    {
        b = 255;
    }
    if (r < 0)
    {
        r = 0;
    }
    if (g < 0)
    {
        g = 0;
    }
    if (b < 0)
    {
        b = 0;
    }
    pixel[0] = r;
    pixel[1] = g;
    pixel[2] = b;
    return pixel32;
}

static void RGB8Planar_T_RGB(unsigned int nWidth, unsigned int nHeight,const unsigned char *pRGBSrc, unsigned char *pRGBDst)
{
    if ((pRGBSrc == nullptr) || (pRGBDst == nullptr))
    {    
        return;
    }
    int offset = nWidth * nHeight;
    for(int i = 0; i < offset; i++)
    {
        pRGBDst[3 * i] = pRGBSrc[i];
        pRGBDst[3 * i + 1] = pRGBSrc[i + offset];
        pRGBDst[3 * i + 2] = pRGBSrc[i + offset * 2];
    }
}

static void YUV422_T_RGB(unsigned int nWidth, unsigned int nHeight,const unsigned char *pYUVSrc, unsigned char *pRGBDst)
{
    unsigned int in, out = 0;
    unsigned int pixel_16;
    unsigned char pixel_24[3];
    unsigned int pixel32;
    unsigned int offset = nWidth * nHeight;
    int y0, u, y1, v;

    if ((pYUVSrc == nullptr) || (pRGBDst == nullptr))
    {  
        return;
    }
    for (in = 0; in < nWidth * nHeight * 2; in += 4)
    {
        pixel_16 =
            pYUVSrc[in + 3] << 24 |
            pYUVSrc[in + 2] << 16 |
            pYUVSrc[in + 1] << 8 |
            pYUVSrc[in + 0];
        y0 = (pixel_16 & 0x000000ff);
        u = (pixel_16 & 0x0000ff00) >> 8;
        y1 = (pixel_16 & 0x00ff0000) >> 16;
        v = (pixel_16 & 0xff000000) >> 24;
        pixel32 = CoverYuv422_T_RGB_Pixel(y0, u, v);
        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        pRGBDst[out++] = pixel_24[0];
        pRGBDst[out++] = pixel_24[1];
        pRGBDst[out++] = pixel_24[2];
        pixel32 = CoverYuv422_T_RGB_Pixel(y1, u, v);
        pixel_24[0] = (pixel32 & 0x000000ff);
        pixel_24[1] = (pixel32 & 0x0000ff00) >> 8;
        pixel_24[2] = (pixel32 & 0x00ff0000) >> 16;
        pRGBDst[out++] = pixel_24[0];
        pRGBDst[out++] = pixel_24[1];
        pRGBDst[out++] = pixel_24[2];
    }
    return;
}

#endif