#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <filesystem>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

// 装甲板结构体定义
typedef struct Armor
{
    float x1;
    float y1;
    float x2;
    float y2;
    float x3;
    float y3;
    float x4;
    float y4;
    float score;
    int label;
} armor;

// 定义颜色数组，用于可视化不同类别
const std::vector<cv::Scalar> COLORS = {
    cv::Scalar(255, 0, 0),     // 蓝色
    cv::Scalar(0, 255, 0),     // 绿色
    cv::Scalar(0, 0, 255),     // 红色
    cv::Scalar(255, 255, 0),   // 青色
    cv::Scalar(255, 0, 255),   // 洋红色
    cv::Scalar(0, 255, 255),   // 黄色
};

// 类别名称映射
const std::vector<std::string> CLASS_NAMES = {
    "armor_sentry_blue",       // 0
    "armor_sentry_red",        // 1
    "armor_sentry_none",       // 2
    "armor_hero_blue",         // 3
    "armor_hero_red",          // 4
    "armor_hero_none",         // 5
    "armor_engine_blue",       // 6
    "armor_engine_red",        // 7
    "armor_engine_none",       // 8
    "armor_infantry_3_blue",   // 9
    "armor_infantry_3_red",    // 10
    "armor_infantry_3_none",   // 11
    "armor_infantry_4_blue",   // 12
    "armor_infantry_4_red",    // 13
    "armor_infantry_4_none",   // 14
    "armor_infantry_5_blue",   // 15
    "armor_infantry_5_red",    // 16
    "armor_infantry_5_none",   // 17
    "armor_outpost_blue",      // 18
    "armor_outpost_red",       // 19
    "armor_outpost_none",      // 20
    "armor_base_blue",         // 21
    "armor_base_red",          // 22
    "armor_infantry_Big_3_blue", // 23
    "armor_infantry_Big_3_red",  // 24
    "armor_infantry_Big_3_none", // 25
    "armor_infantry_Big_4_blue", // 26
    "armor_infantry_Big_4_red",  // 27
    "armor_infantry_Big_4_none", // 28
    "armor_infantry_Big_5_blue", // 29
    "armor_infantry_Big_5_red",  // 30
    "armor_infantry_Big_5_none", // 31
    "armor_base_purple",       // 32
    "yindaodeng"               // 33
};

// 简化的类别名称映射（用于显示）
const std::vector<std::string> DISPLAY_NAMES = {
    "哨兵蓝",       // 0
    "哨兵红",       // 1
    "哨兵无",       // 2
    "英雄蓝",       // 3
    "英雄红",       // 4
    "英雄无",       // 5
    "工程蓝",       // 6
    "工程红",       // 7
    "工程无",       // 8
    "3号蓝",        // 9
    "3号红",        // 10
    "3号无",        // 11
    "4号蓝",        // 12
    "4号红",        // 13
    "4号无",        // 14
    "5号蓝",        // 15
    "5号红",        // 16
    "5号无",        // 17
    "前哨蓝",       // 18
    "前哨红",       // 19
    "前哨无",       // 20
    "基地蓝",       // 21
    "基地红",       // 22
    "大3蓝",        // 23
    "大3红",        // 24
    "大3无",        // 25
    "大4蓝",        // 26
    "大4红",        // 27
    "大4无",        // 28
    "大5蓝",        // 29
    "大5红",        // 30
    "大5无",        // 31
    "基地紫",       // 32
    "引导灯"        // 33
};

// 预处理函数，将图像数据转换为模型输入格式
inline void preprocess(cv::Mat &image, ov::Tensor &tensor)
{
    // 确保图像是以浮点格式
    cv::Mat float_image;
    image.convertTo(float_image, CV_32FC3, 1.0/255.0);  // 归一化到[0,1]
    
    int img_w = float_image.cols;
    int img_h = float_image.rows;
    int channels = 3;

    auto data = tensor.data<float>();

    for (size_t c = 0; c < channels; c++)
    {
        for (size_t h = 0; h < img_h; h++)
        {
            for (size_t w = 0; w < img_w; w++)
            {
                // OpenCV默认是BGR格式，将通道顺序从BGR转为RGB
                data[c * img_w * img_h + h * img_w + w] =
                    float_image.at<cv::Vec3f>(h, w)[2 - c];
            }
        }
    }
}

// 计算两个装甲板的IOU
inline float cal_iou(const Armor a, const Armor b) {
    // 计算两个四边形的外接矩形
    int ax_min = std::min(std::min(std::min(a.x1, a.x2), a.x3), a.x4);
    int ax_max = std::max(std::max(std::max(a.x1, a.x2), a.x3), a.x4);
    int ay_min = std::min(std::min(std::min(a.y1, a.y2), a.y3), a.y4);
    int ay_max = std::max(std::max(std::max(a.y1, a.y2), a.y3), a.y4);

    int bx_min = std::min(std::min(std::min(b.x1, b.x2), b.x3), b.x4);
    int bx_max = std::max(std::max(std::max(b.x1, b.x2), b.x3), b.x4);
    int by_min = std::min(std::min(std::min(b.y1, b.y2), b.y3), b.y4);
    int by_max = std::max(std::max(std::max(b.y1, b.y2), b.y3), b.y4);

    float max_x = std::max(ax_min, bx_min);
    float min_x = std::min(ax_max, bx_max);
    float max_y = std::max(ay_min, by_min);
    float min_y = std::min(ay_max, by_max);

    if(min_x <= max_x || min_y <= max_y)
        return 0;
    
    float over_area = (min_x - max_x) * (min_y - max_y);

    float area_a = (ax_max - ax_min) * (ay_max - ay_min);
    float area_b = (bx_max - bx_min) * (by_max - by_min);
    float iou = over_area / (area_a + area_b - over_area);
    return iou;
}

// NMS处理函数
inline void nms(float* result, float conf_thr, float iou_thr, std::vector<Armor>& armors, int class_nums) {
    // 遍历result，如果conf大于阈值conf_thr，则放入armors
    for(int i = 0; i < 25200; ++i) {
        if(result[8 + i * class_nums] >= conf_thr) {
            Armor temp;
            // 将四个角点放入
            temp.x1 = int(result[0 + i * class_nums]);
            temp.y1 = int(result[1 + i * class_nums]);
            temp.x2 = int(result[2 + i * class_nums]);
            temp.y2 = int(result[3 + i * class_nums]);
            temp.x3 = int(result[4 + i * class_nums]);
            temp.y3 = int(result[5 + i * class_nums]);
            temp.x4 = int(result[6 + i * class_nums]);
            temp.y4 = int(result[7 + i * class_nums]);

            // 找到最大的条件类别概率并乘上conf作为类别概率
            float cls = result[i * class_nums + 9];
            int cnt = 0;
            for(int j = i * class_nums + 10; j < i * class_nums + class_nums; ++j) {
                if(cls < result[j]) {
                    cls = result[j];
                    cnt = (j - 9) % (class_nums - 9);  // 修正类别索引计算
                }
            }
            cls *= result[8 + i * class_nums];
            temp.score = cls;
            temp.label = cnt;
            armors.push_back(temp);
        }
    }
    
    // 对得到的armor按score进行降序排序
    std::sort(armors.begin(), armors.end(), [](Armor a, Armor b) { return a.score > b.score; });

    // 只保留置信度最高的一个装甲板
    if (armors.size() > 1) {
        armors.resize(1);
    }

    //按iou_thr将重合度高的armor进行筛掉
    for(int i = 0;i < int(armors.size());++i)
    {
        for(int j = i + 1;j < int(armors.size());++j)
            //如果与当前的框iou大于阈值则erase掉
            if(cal_iou(armors[i], armors[j]) > iou_thr)
            {
                armors.erase(armors.begin() + j);
                --j;//万年不见--
            }
    }
}
