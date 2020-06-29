/*
   匹配和可视化 
   Reference:https://github.com/robertwgh/ezSIFT
*/
#include "image_utility.h"
#include "common.h"
#include "sift.h"
#include "image.h"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctype.h>
#include <limits>

namespace sift {

/**********************************************
********        SIFT关键点匹配        ***********
***********************************************/

/****************** 辅助函数 *******************/

// 重复匹配对判断
bool same_match_pair(const MatchPair &first, const MatchPair &second)
{
    if (first.c1 == second.c1 && first.r1 == second.r1 &&
        first.c2 == second.c2 && first.r2 == second.r2)
        return true;
    else
        return false;
}

/****************  匹配核心函数 *****************/

// 关键点匹配: 基于欧氏距离, 暴力搜索
int match_keypoints(std::list<SiftKeypoint> &kpt_list1, std::list<SiftKeypoint> &kpt_list2, std::list<MatchPair> &match_list)
/*
    Parameters:
        kpt_list1: 图1的SIFT关键点列表
        kpt_list2: 图2的SIFT关键点列表
        match_list: 匹配列表
*/
{
    std::list<SiftKeypoint>::iterator kpt1;
    std::list<SiftKeypoint>::iterator kpt2;

    for (kpt1 = kpt_list1.begin(); kpt1 != kpt_list1.end(); kpt1++) {
        // 待匹配关键点坐标.
        int r1 = (int)kpt1->r;
        int c1 = (int)kpt1->c;

        float *descr1 = kpt1->descriptors;
        float score1 = (std::numeric_limits<float>::max)(); // highest score
        float score2 = (std::numeric_limits<float>::max)(); // 2nd highest score

        // 搜索的关键点坐标
        int r2 = 0, c2 = 0;
        for (kpt2 = kpt_list2.begin(); kpt2 != kpt_list2.end(); kpt2++) {
            float score = 0;
            float *descr2 = kpt2->descriptors;
            float dif;
            for (int i = 0; i < DEGREE_OF_DESCRIPTORS; i++) {
                dif = descr1[i] - descr2[i];
                score += dif * dif;
            }

            if (score < score1) {
                score2 = score1;
                score1 = score;
                r2 = (int)kpt2->r;
                c2 = (int)kpt2->c;
            }
            else if (score < score2) {
                score2 = score;
            }
        }

        // 阈值筛选 & 匹配
        if (fast_sqrt_f(score1 / score2) < SIFT_MATCH_NNDR_THR)
        {
            MatchPair mp;
            mp.r1 = r1;
            mp.c1 = c1;
            mp.r2 = r2;
            mp.c2 = c2;

            match_list.push_back(mp);
        }
    }
    // 重复匹配对处理
    match_list.unique(same_match_pair);

    // 输出关键点匹配对信息
    std::list<MatchPair>::iterator p;
    int match_idx = 0;
    printf("\nSIFT关键点匹配信息:\n");
    for (p = match_list.begin(); p != match_list.end(); p++) {
        printf("\tNO. %3d: (%4d, %4d) -> (%4d, %4d)\n", match_idx, p->r1, p->c1, p->r2, p->c2);
        match_idx++;
    }

    return 0;
}




/**********************************************
********        SIFT关键点可视化      ***********
***********************************************/

/****************** 辅助函数 *******************/

// 像素置红
void setPixelRed(ImagePPM *img, int r, int c)
{
    if ((r >= 0) && (r < img->h) && (c >= 0) && (c < img->w)) {
        img->img_r[r * img->w + c] = 255;
        img->img_g[r * img->w + c] = 0;
        img->img_b[r * img->w + c] = 0;
    }
}

// 画红圈
// http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
void rasterCircle(ImagePPM *imgPPM, int r, int c, int radius)
{
    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    int x0 = r;
    int y0 = c;

    setPixelRed(imgPPM, x0, y0 + radius);
    setPixelRed(imgPPM, x0, y0 - radius);
    setPixelRed(imgPPM, x0 + radius, y0);
    setPixelRed(imgPPM, x0 - radius, y0);

    while (x < y) {
        // ddF_x == 2 * x + 1;
        // ddF_y == -2 * y;
        // f == x*x + y*y - radius*radius + 2*x - y + 1;
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        setPixelRed(imgPPM, x0 + x, y0 + y);
        setPixelRed(imgPPM, x0 - x, y0 + y);
        setPixelRed(imgPPM, x0 + x, y0 - y);
        setPixelRed(imgPPM, x0 - x, y0 - y);
        setPixelRed(imgPPM, x0 + y, y0 + x);
        setPixelRed(imgPPM, x0 - y, y0 + x);
        setPixelRed(imgPPM, x0 + y, y0 - x);
        setPixelRed(imgPPM, x0 - y, y0 - x);
    }
}

// 标记梯度方向
void draw_red_orientation(ImagePPM *imgPPM, int x, int y, float ori, int cR)
{
    int xe = (int)(x + cos(ori) * cR), ye = (int)(y + sin(ori) * cR);
    // Bresenham's line algorithm
    int dx = abs(xe - x), sx = x < xe ? 1 : -1;
    int dy = -abs(ye - y), sy = y < ye ? 1 : -1;
    int err = dx + dy, e2; /* error value e_xy */

    for (;;) { /* loop */
        setPixelRed(imgPPM, y, x);
        if (x == xe && y == ye)
            break;
        e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x += sx;
        } /* e_xy+e_x > 0 */
        if (e2 <= dx) {
            err += dx;
            y += sy;
        } /* e_xy+e_y < 0 */
    }
}

// 可视化结果输出
void write_rgb2ppm(const char *filename, unsigned char *r, unsigned char *g, unsigned char *b, int w, int h)
{
    FILE *out_file;
    int i;

    unsigned char *obuf =
        (unsigned char *)malloc(3 * w * h * sizeof(unsigned char));

    for (i = 0; i < w * h; i++) {
        obuf[3 * i + 0] = r[i];
        obuf[3 * i + 1] = g[i];
        obuf[3 * i + 2] = b[i];
    }
    out_file = fopen(filename, "wb");
    fprintf(out_file, "P6\n");
    fprintf(out_file, "%d %d\n255\n", w, h);
    fwrite(obuf, sizeof(unsigned char), 3 * w * h, out_file);
    fclose(out_file);
    free(obuf);
}

/**************** 可视化核心函数 *****************/

// SIFT关键点可视化
void draw_keypoints_to_ppm_file(const char *out_filename, const Image<unsigned char> &image, std::list<SiftKeypoint> kpt_list)
/*
    Parameters:
        out_filename: 可视化结果文件名
        Image: 可视化输入图
        kpt_list: SIFT关键点列表
*/
{
    std::list<SiftKeypoint>::iterator it;
    ImagePPM imgPPM;
    int w = image.w;
    int h = image.h;
    int r, c;

    // 关键点可视化半径: cR = sigma * 4 * (2^O)
    int cR;

    // 初始化PPM图像
    imgPPM.w = w;
    imgPPM.h = h;
    imgPPM.img_r = new unsigned char[w * h];
    imgPPM.img_g = new unsigned char[w * h];
    imgPPM.img_b = new unsigned char[w * h];

    // 复制输入灰度图片 & 转为rgb三通道
    int i, j;
    unsigned char *data = image.data;
    for (i = 0; i < h; i++) {
        for (j = 0; j < w; j++) {
            imgPPM.img_r[i * w + j] = data[i * w + j];
            imgPPM.img_g[i * w + j] = data[i * w + j];
            imgPPM.img_b[i * w + j] = data[i * w + j];
        }
    }

    // 关键点可视化
    for (it = kpt_list.begin(); it != kpt_list.end(); it++) {
        // 半径 & 位置
        cR = (int)it->scale;
        if (cR <= 1) {
            cR = 1; // 小尺度关键点可视化半径为1
        }
        r = (int)it->r;
        c = (int)it->c;
        // 画红圈
        rasterCircle(&imgPPM, r, c, 1);
        rasterCircle(&imgPPM, r, c, cR+1);
        float ori = it->ori;
        // 标记梯度方向
        draw_red_orientation(&imgPPM, c, r, ori, cR);
    }

    // 可视化结果输出
    write_rgb2ppm(out_filename, imgPPM.img_r, imgPPM.img_g, imgPPM.img_b, w, h);

    // 内存释放
    delete[] imgPPM.img_r;
    delete[] imgPPM.img_g;
    delete[] imgPPM.img_b;
    imgPPM.img_r = imgPPM.img_g = imgPPM.img_b = nullptr;
}




/**********************************************
********    SIFT关键点匹配结果可视化    ***********
***********************************************/

/****************** 辅助函数 *******************/

// 水平合并两张图片: 用于SIFT关键点匹配结果可视化
int combine_image(Image<unsigned char> &out_image, const Image<unsigned char> &image1, const Image<unsigned char> &image2)
{
    int w1 = image1.w;
    int h1 = image1.h;
    int w2 = image2.w;
    int h2 = image2.h;
    int dstW = w1 + w2;
    int dstH = MAX(h1, h2);

    out_image.init(dstW, dstH);

    unsigned char *srcData1 = image1.data;
    unsigned char *srcData2 = image2.data;
    unsigned char *dstData = out_image.data;

    for (int r = 0; r < dstH; r++) {
        if (r < h1) {
            memcpy(dstData, srcData1, w1 * sizeof(unsigned char));
        }
        else {
            memset(dstData, 0, w1 * sizeof(unsigned char));
        }
        dstData += w1;

        if (r < h2) {
            memcpy(dstData, srcData2, w2 * sizeof(unsigned char));
        }
        else {
            memset(dstData, 0, w2 * sizeof(unsigned char));
        }
        dstData += w2;
        srcData1 += w1;
        srcData2 += w2;
    }

    return 0;
}

// 画线(rgb): 用于可视化匹配对
int draw_line_to_rgb_image(unsigned char *&data, int w, int h, MatchPair &mp)
{
    int r1 = mp.r1;
    int c1 = mp.c1;
    int r2 = mp.r2;
    int c2 = mp.c2;

    // 根据始末点获得线段表达式, 逐点标蓝
    float k = (float)(r2 - r1) / (float)(c2 - c1);
    for (int c = c1; c < c2; c++) {
        // 根据线段表达式由c求r
        int r = (int)(k * (c - c1) + r1);
        // 标蓝
        data[r * w * 3 + 3 * c] = 0;
        data[r * w * 3 + 3 * c + 1] = 0;
        data[r * w * 3 + 3 * c + 2] = 255;
    }
    return 0;
}

// 写入ppm
void write_ppm(const char *filename, unsigned char *data, int w, int h)
{
    FILE *fp;
    if ((fp = fopen(filename, "wb")) == NULL) {
        printf("Cannot write to file %s\n", filename);
        return;
    }

    /* Write header */
    fprintf(fp, "P6\n");
    fprintf(fp, "%d %d\n", w, h);
    fprintf(fp, "255\n");

    fwrite(data, sizeof(unsigned char), w * h * 3, fp);
    fclose(fp);
}

/**************** 可视化核心函数 *****************/

// 可视化SIFT关键点匹配对
int draw_match_lines_to_ppm_file(const char *filename, Image<unsigned char> &image1, Image<unsigned char> &image2, std::list<MatchPair> &match_list)
/*
    Parameters:
        filename: 可视化输出文件名
        image1: 匹配输入图像1
        image2: 匹配输入图像2
        match_list: SIFT关键点匹配对列表
*/
{   
    // 水平合并两张图片
    Image<unsigned char> tmpImage;
    combine_image(tmpImage, image1, image2);

    // 初始化结果图片
    int w = tmpImage.w;
    int h = tmpImage.h;
    unsigned char *srcData = tmpImage.data;
    unsigned char *dstData = new unsigned char[w * h * 3];
    for (int i = 0; i < w * h; i++) {
        dstData[i * 3 + 0] = srcData[i];
        dstData[i * 3 + 1] = srcData[i];
        dstData[i * 3 + 2] = srcData[i];
    }

    // 可视化匹配结果
    std::list<MatchPair>::iterator p;
    for (p = match_list.begin(); p != match_list.end(); p++) {
        MatchPair mp;
        mp.r1 = p->r1;
        mp.c1 = p->c1;
        mp.r2 = p->r2;
        mp.c2 = p->c2 + image1.w;
        draw_line_to_rgb_image(dstData, w, h, mp);
    }
    // 可视化结果输出
    write_ppm(filename, dstData, w, h);
    // 内存释放
    delete[] dstData;
    dstData = nullptr;
    return 0;
}

} // end namespace sift
