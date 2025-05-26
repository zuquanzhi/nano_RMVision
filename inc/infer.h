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
    float x;      // 中心点x坐标
    float y;      // 中心点y坐标
    float width;  // 宽度
    float height; // 高度
    float score;  // 置信度
    int label;    // 类别
} Armor, Detection;  // 添加别名Detection使其兼容cal_iou和nms函数

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
    "target_area",           // 0
    "active_target_area",    // 1
    "arrow_lightbar",        // 2
    "active_lightbar",       // 3
    "r_logo",                // 4
    "ten_score"              // 5
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
inline float cal_iou(const Detection& a, const Detection& b) {
    // 计算两个框的边界
    float a_x1 = a.x - a.width / 2;
    float a_y1 = a.y - a.height / 2;
    float a_x2 = a.x + a.width / 2;
    float a_y2 = a.y + a.height / 2;
    
    float b_x1 = b.x - b.width / 2;
    float b_y1 = b.y - b.height / 2;
    float b_x2 = b.x + b.width / 2;
    float b_y2 = b.y + b.height / 2;
    
    // 计算交集区域
    float inter_x1 = std::max(a_x1, b_x1);
    float inter_y1 = std::max(a_y1, b_y1);
    float inter_x2 = std::min(a_x2, b_x2);
    float inter_y2 = std::min(a_y2, b_y2);
    
    // 检查是否有交集
    if(inter_x1 >= inter_x2 || inter_y1 >= inter_y2)
        return 0;
    
    float inter_area = (inter_x2 - inter_x1) * (inter_y2 - inter_y1);
    float a_area = a.width * a.height;
    float b_area = b.width * b.height;
    
    // 计算IOU
    float iou = inter_area / (a_area + b_area - inter_area);
    return iou;
}

// NMS处理函数
inline void nms(float* result, float conf_thr, float iou_thr, std::vector<Armor>& detections, int class_nums) {
    // 遍历result，如果conf大于阈值conf_thr，则放入detections
    for(int i = 0; i < 25200; ++i) {
        if(result[4 + i * class_nums] >= conf_thr) {  // YOLOv5中第5个元素是置信度
            Armor temp;
            // 直接获取中心点和宽高
            temp.x = result[0 + i * class_nums];      // 中心点x
            temp.y = result[1 + i * class_nums];      // 中心点y
            temp.width = result[2 + i * class_nums];  // 宽度
            temp.height = result[3 + i * class_nums]; // 高度

            // 找到最大的条件类别概率并乘上conf作为类别概率
            float max_cls_prob = result[i * class_nums + 5];   // 类别预测从第6个元素开始
            int class_idx = 0;
            
            // 计算类别索引
            for(int j = i * class_nums + 5; j < i * class_nums + class_nums; ++j) {
                if(max_cls_prob < result[j]) {
                    max_cls_prob = result[j];
                    class_idx = j - (i * class_nums + 5);  // 计算类别索引
                }
            }
            
            float conf_score = max_cls_prob * result[4 + i * class_nums];  // 类别概率 = 最大类别概率 * 置信度
            temp.score = conf_score;
            temp.label = class_idx;
            detections.push_back(temp);
        }
    }
    
    // 对得到的detection按score进行降序排序
    std::sort(detections.begin(), detections.end(), [](const Armor& a, const Armor& b) { return a.score > b.score; });

    // 标准YOLOv5 NMS处理
    for(int i = 0; i < int(detections.size()); ++i) {
        for(int j = i + 1; j < int(detections.size()); ++j) {
            // 如果与当前的框iou大于阈值则删除
            if(cal_iou(detections[i], detections[j]) > iou_thr) {
                detections.erase(detections.begin() + j);
                --j; // 删除元素后，索引减一继续检查
            }
        }
    }
}
