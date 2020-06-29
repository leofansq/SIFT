#ifndef IMAGE_UTILITY_H
#define IMAGE_UTILITY_H

#include <list>

namespace sift {

/**********************************************
********            对象定义         ***********
***********************************************/

struct SiftKeypoint;
struct MatchPair;

template <typename T>
class Image;

struct ImagePPM {
    int w;
    int h;
    unsigned char *img_r;
    unsigned char *img_g;
    unsigned char *img_b;
};

/**********************************************
********            可视化函数         ***********
***********************************************/

// SIFT关键点可视化
void draw_keypoints_to_ppm_file(const char *out_filename, const Image<unsigned char> &image, std::list<SiftKeypoint> kpt_list);

// 可视化SIFT关键点匹配对
int draw_match_lines_to_ppm_file(const char *filename, Image<unsigned char> &image1, Image<unsigned char> &image2, std::list<MatchPair> &match_list);


/**********************************************
********           图像基础函数        ***********
***********************************************/

// 获取像素值(unsigned char)
inline unsigned char get_pixel(unsigned char *imageData, int w, int h, int r, int c)
{
    unsigned char val;
    if (c >= 0 && c < w && r >= 0 && r < h) {
        val = imageData[r * w + c];
    }
    else if (c < 0) {
        val = imageData[r * w];
    }
    else if (c >= w) {
        val = imageData[r * w + w - 1];
    }
    else if (r < 0) {
        val = imageData[c];
    }
    else if (r >= h) {
        val = imageData[(h - 1) * w + c];
    }
    else {
        val = 0;
    }
    return val;
}

// 获取像素值(float)
inline float get_pixel_f(float *imageData, int w, int h, int r, int c)
{
    float val;
    if (c >= 0 && c < w && r >= 0 && r < h) {
        val = imageData[r * w + c];
    }
    else if (c < 0) {
        val = imageData[r * w];
    }
    else if (c >= w) {
        val = imageData[r * w + w - 1];
    }
    else if (r < 0) {
        val = imageData[c];
    }
    else if (r >= h) {
        val = imageData[(h - 1) * w + c];
    }
    else {
        val = 0.0f;
    }
    return val;
}

} // end namespace sift

#endif // IMAGE_UTILITY_H
