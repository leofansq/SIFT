/*
SIFT 关键点检测 & 匹配
 */

#include "sift.h"

#include <iostream>
#include <list>


int main(int argc, char *argv[])
{
    // 读取图片文件参数
    if (argc != 3) {
        std::cerr << "请前后依次输入两个图像文件名\n" << "命令格式如: image_match img1 img2" << std::endl;
        return -1;
    }
    char file1[255];
    char file2[255];
    memcpy(file1, argv[1], sizeof(char) * strlen(argv[1]));
    file1[strlen(argv[1])] = 0;
    memcpy(file2, argv[2], sizeof(char) * strlen(argv[2]));
    file2[strlen(argv[2])] = 0;


    // 读入两张待匹配图片
    sift::Image<unsigned char> image1, image2;
    // 异常检测: 图片文件异常
    if (image1.read_pgm(file1) != 0) {
        std::cerr << "Failed to open input image1!" << std::endl;
        return -1;
    }
    if (image2.read_pgm(file2) != 0) {
        std::cerr << "Failed to open input image2!" << std::endl;
        return -1;
    }
    // 成功读入图片, 输出信息
    std::cout << "\n图片加载 --- Done" << std::endl;

    // SIFT关键点检测
    std::list<sift::SiftKeypoint> kpt_list1, kpt_list2;

    std::cout << "\n图片1 SIFT特征检测 --- ";
    sift::sift_cpu(image1, kpt_list1, true);
    std::cout << "Done" << std::endl;
    
    std::cout << "\n图片2 SIFT特征检测 --- ";
    sift::sift_cpu(image2, kpt_list2, true);
    std::cout << "Done" << std::endl;

    // 关键点匹配
    std::list<sift::MatchPair> match_list;
    sift::match_keypoints(kpt_list1, kpt_list2, match_list);

    // 可视化
    // 关键点可视化
    std::cout << std::endl;
    sift::draw_keypoints_to_ppm_file("SIFT_KP_1.ppm", image1, kpt_list1);
    std::cout << "Image 1 中共检测到SIFT特征: " << static_cast<unsigned int>(kpt_list1.size()) << "个.\t可视化结果为SIFT_KP_1.ppm" << std::endl;
    sift::draw_keypoints_to_ppm_file("SIFT_KP_2.ppm", image2, kpt_list2);
    std::cout << "Image 2 中共检测到SIFT特征: " << static_cast<unsigned int>(kpt_list2.size()) << "个.\t可视化结果为SIFT_KP_2.ppm" << std::endl;

    // 匹配结果可视化
    sift::draw_match_lines_to_ppm_file("SIFT_match.ppm", image1, image2, match_list);
    std::cout << "共匹配SIFT特征点: "<< static_cast<unsigned int>(match_list.size()) << "对.\t\t可视化结果为SIFT_match.ppm"<< std::endl;

    return 0;
}
