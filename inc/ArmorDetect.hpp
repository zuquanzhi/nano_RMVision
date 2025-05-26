#ifndef ARMOR_DETECT_HPP
#define ARMOR_DETECT_HPP

#include <openvino/openvino.hpp>
#include <infer.h>

// 模型推理处理类
class ArmorDetector {
private:
    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;
    std::string model_path;
    float scale;
    int padding_x, padding_y;
    
public:
    // 构造函数
    ArmorDetector(const std::string& model_path) : model_path(model_path) {}
    
    // 初始化模型
    bool init() {
        try {
            // 读取模型
            auto model = core.read_model(model_path);
            
            // 优先使用GPU，失败则回退到CPU
            try {
                compiled_model = core.compile_model(model, "GPU");
                std::cout << "使用GPU进行推理" << std::endl;
            } catch (const std::exception&) {
                compiled_model = core.compile_model(model, "CPU");
                std::cout << "使用CPU进行推理" << std::endl;
            }
            
            // 创建推理请求
            infer_request = compiled_model.create_infer_request();
            return true;
        } catch (const std::exception& e) {
            std::cerr << "模型初始化失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    // 处理图像并进行推理
    bool process(cv::Mat& image, std::vector<Armor>& armors, float conf_thr = 0.5, float iou_thr = 0.45) {
        try {
            // 图像预处理
            cv::Mat processed_image;
            preprocess_image(image, processed_image);
            
            // 准备输入张量
            ov::Tensor input_tensor = infer_request.get_input_tensor();
            std::cout << "输入张量形状: " << input_tensor.get_shape() << std::endl;
            preprocess(processed_image, input_tensor);
            
            // 执行推理
            auto infer_start = std::chrono::steady_clock::now();
            infer_request.infer();
            auto infer_end = std::chrono::steady_clock::now();
            std::cout << "推理时间: " << std::chrono::duration<double>(infer_end - infer_start).count() * 1000 << " 毫秒" << std::endl;
            
            // 获取输出并处理
            ov::Tensor output_tensor = infer_request.get_output_tensor();
            std::cout << "输出张量形状: " << output_tensor.get_shape() << std::endl;
            auto result = output_tensor.data<float>();
            
            // 执行NMS获取装甲板检测结果（使用标准YOLO格式）
            nms(result, conf_thr, iou_thr, armors, 43);
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "处理失败: " << e.what() << std::endl;
            return false;
        }
    }
    
    // 将检测结果可视化到原始图像上
    void visualize(cv::Mat& image, const std::vector<Armor>& armors, const std::string& output_path = "") {
        cv::Mat visualization_image = image.clone();
        
        for (const auto& armor : armors) {
            // 将YOLO格式坐标映射回原始图像
            float x = (armor.x - padding_x) / scale;
            float y = (armor.y - padding_y) / scale;
            float w = armor.width / scale;
            float h = armor.height / scale;
            
            // 计算矩形框的四个角点
            int x1 = std::max(0, std::min(int(x - w/2), image.cols - 1));
            int y1 = std::max(0, std::min(int(y - h/2), image.rows - 1));
            int x2 = std::max(0, std::min(int(x + w/2), image.cols - 1));
            int y2 = std::max(0, std::min(int(y - h/2), image.rows - 1));
            int x3 = std::max(0, std::min(int(x + w/2), image.cols - 1));
            int y3 = std::max(0, std::min(int(y + h/2), image.rows - 1));
            int x4 = std::max(0, std::min(int(x - w/2), image.cols - 1));
            int y4 = std::max(0, std::min(int(y + h/2), image.rows - 1));
            
            // 绘制装甲板
            cv::Scalar color = COLORS[armor.label % COLORS.size()];
            std::vector<cv::Point> polygon = {
                cv::Point(x1, y1), cv::Point(x2, y2), 
                cv::Point(x3, y3), cv::Point(x4, y4)
            };
            
            cv::polylines(visualization_image, std::vector<std::vector<cv::Point>>{polygon}, true, color, 2);
            
            // 绘制角点
            cv::circle(visualization_image, cv::Point(x1, y1), 5, cv::Scalar(0, 0, 255), -1);
            cv::circle(visualization_image, cv::Point(x2, y2), 5, cv::Scalar(0, 255, 0), -1);
            cv::circle(visualization_image, cv::Point(x3, y3), 5, cv::Scalar(255, 0, 0), -1);
            cv::circle(visualization_image, cv::Point(x4, y4), 5, cv::Scalar(255, 255, 0), -1);
            
            // 绘制中心点
            cv::circle(visualization_image, cv::Point(int(x), int(y)), 5, cv::Scalar(255, 255, 255), -1);
            
            // 标签信息
            std::string label = (armor.label < CLASS_NAMES.size() ? 
                                CLASS_NAMES[armor.label] : "class" + std::to_string(armor.label)) + 
                                " " + std::to_string(int(armor.score * 100)) + "%";
            
            cv::putText(visualization_image, label, cv::Point(x1, y1 - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            
            std::cout << "装甲板: 类别=" << label 
                      << ", 中心点=(" << x << "," << y << ")"
                      << ", 宽高=(" << w << "," << h << ")"
                      << ", 置信度=" << armor.score << std::endl;
        }
        
        // 保存结果
        if (!output_path.empty()) {
            cv::imwrite(output_path, visualization_image);
            std::cout << "结果已保存到: " << output_path << std::endl;
        }
        
        // 将可视化结果更新回原始图像
        visualization_image.copyTo(image);
    }
    
private:
    // 图像预处理
    void preprocess_image(cv::Mat& original_image, cv::Mat& processed_image) {
        scale = std::min(float(640) / original_image.cols, float(640) / original_image.rows);
        padding_y = int((640 - original_image.rows * scale) / 2);
        padding_x = int((640 - original_image.cols * scale) / 2);
        
        cv::resize(original_image, processed_image, cv::Size(original_image.cols * scale, original_image.rows * scale));
        cv::copyMakeBorder(processed_image, processed_image, padding_y, padding_y, padding_x, padding_x, cv::BORDER_CONSTANT, cv::Scalar(144, 144, 144));
    }
};

#endif // ARMOR_DETECT_HPP