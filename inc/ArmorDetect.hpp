#ifndef ARMOR_DETECT_HPP
#define ARMOR_DETECT_HPP

#include <openvino/openvino.hpp>
#include "infer.h"
#include "ArmorExtension.hpp"
#include "ConfigManager.h"

// 模型推理处理类
class ArmorDetector {
private:
    ov::Core core;
    ov::CompiledModel compiled_model;
    ov::InferRequest infer_request;
    std::string model_path;
    float scale;
    int padding_x, padding_y;
    bool enable_corner_correction;
    bool debug_mode;
    
    // 角点优化参数 - 从配置文件中读取
    float ROI_EXPAND_RATIO;
    float LENGTH_EXTEND_FACTOR;
    int MIN_CONTOUR_POINTS;
    float MAX_DEVIATION_RATIO;
    float MIN_LIGHTBAR_RATIO;
    float BRIGHTNESS_PERCENTILE;
    float COLOR_WEIGHT;
    
    // 角点验证参数 - 从配置文件中读取
    float MIN_ASPECT_RATIO;
    float MAX_ASPECT_RATIO;
    float PARALLEL_TOLERANCE_DEG;
    float MIN_EDGE_RATIO;
    float MAX_EDGE_RATIO;
    float MIN_DIAG_RATIO;
    float MAX_DIAG_RATIO;
    float MIN_AREA_RATIO;
    float MAX_AREA_RATIO;
    
public:
    // 构造函数
    ArmorDetector(const std::string& model_path, ConfigManager* configManager = nullptr) 
    : model_path(model_path), enable_corner_correction(false), debug_mode(false) {
        // 默认角点优化参数值
        ROI_EXPAND_RATIO = 0.6f;
        LENGTH_EXTEND_FACTOR = 1.7f;
        MIN_CONTOUR_POINTS = 6;
        MAX_DEVIATION_RATIO = 0.95f;
        MIN_LIGHTBAR_RATIO = 2.0f;
        BRIGHTNESS_PERCENTILE = 0.05f;
        COLOR_WEIGHT = 0.1f;
        
        // 默认角点验证参数值
        MIN_ASPECT_RATIO = 1.0f;
        MAX_ASPECT_RATIO = 5.0f;
        PARALLEL_TOLERANCE_DEG = 3.0f;
        MIN_EDGE_RATIO = 0.8f;
        MAX_EDGE_RATIO = 1.2f;
        MIN_DIAG_RATIO = 0.8f;
        MAX_DIAG_RATIO = 1.2f;
        MIN_AREA_RATIO = 0.7f;
        MAX_AREA_RATIO = 1.3f;
        
        // 如果提供了配置管理器，则从配置中加载参数
        if (configManager != nullptr) {
            loadConfig(configManager);
        }
    }
    
    // 从配置管理器读取参数
    void loadConfig(ConfigManager* configManager) {
        if (!configManager) return;
        
        bool corner_optimization;
        if (configManager->readBool("corner_optimization", corner_optimization)) {
            enable_corner_correction = corner_optimization;
        }
        
        double value;
        // 读取角点优化基本参数
        if (configManager->readDouble("ROI_EXPAND_RATIO", value)) {
            ROI_EXPAND_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("LENGTH_EXTEND_FACTOR", value)) {
            LENGTH_EXTEND_FACTOR = static_cast<float>(value);
        }
        
        int int_value;
        if (configManager->readInt("MIN_CONTOUR_POINTS", int_value)) {
            MIN_CONTOUR_POINTS = int_value;
        }
        
        if (configManager->readDouble("MAX_DEVIATION_RATIO", value)) {
            MAX_DEVIATION_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MIN_LIGHTBAR_RATIO", value)) {
            MIN_LIGHTBAR_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("BRIGHTNESS_PERCENTILE", value)) {
            BRIGHTNESS_PERCENTILE = static_cast<float>(value);
        }
        
        if (configManager->readDouble("COLOR_WEIGHT", value)) {
            COLOR_WEIGHT = static_cast<float>(value);
        }
        
        // 读取角点验证参数
        if (configManager->readDouble("MIN_ASPECT_RATIO", value)) {
            MIN_ASPECT_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MAX_ASPECT_RATIO", value)) {
            MAX_ASPECT_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("PARALLEL_TOLERANCE_DEG", value)) {
            PARALLEL_TOLERANCE_DEG = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MIN_EDGE_RATIO", value)) {
            MIN_EDGE_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MAX_EDGE_RATIO", value)) {
            MAX_EDGE_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MIN_DIAG_RATIO", value)) {
            MIN_DIAG_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MAX_DIAG_RATIO", value)) {
            MAX_DIAG_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MIN_AREA_RATIO", value)) {
            MIN_AREA_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readDouble("MAX_AREA_RATIO", value)) {
            MAX_AREA_RATIO = static_cast<float>(value);
        }
        
        if (configManager->readInt("debug_status", int_value)) {
            debug_mode = (int_value == 1);
        }
        
        std::cout << "角点优化配置已加载: " 
                  << "启用=" << (enable_corner_correction ? "是" : "否") 
                  << ", ROI扩展比例=" << ROI_EXPAND_RATIO
                  << ", 灯条扩展系数=" << LENGTH_EXTEND_FACTOR
                  << ", 最小轮廓点数=" << MIN_CONTOUR_POINTS
                  << ", 最大偏差比例=" << MAX_DEVIATION_RATIO
                  << ", 最小灯条比例=" << MIN_LIGHTBAR_RATIO
                  << ", 亮度百分位阈值=" << BRIGHTNESS_PERCENTILE
                  << ", 颜色权重=" << COLOR_WEIGHT
                  << std::endl;
        
        std::cout << "角点验证参数已加载: "
                  << "宽高比范围=" << MIN_ASPECT_RATIO << "-" << MAX_ASPECT_RATIO
                  << ", 平行度容差=" << PARALLEL_TOLERANCE_DEG << "度"
                  << ", 对边长度比范围=" << MIN_EDGE_RATIO << "-" << MAX_EDGE_RATIO
                  << ", 对角线比范围=" << MIN_DIAG_RATIO << "-" << MAX_DIAG_RATIO
                  << ", 面积比范围=" << MIN_AREA_RATIO << "-" << MAX_AREA_RATIO
                  << ", 调试模式=" << (debug_mode ? "开启" : "关闭")
                  << std::endl;
    }
    
    // 设置单个角点优化参数
    void setROIExpandRatio(float value) { ROI_EXPAND_RATIO = value; }
    void setLengthExtendFactor(float value) { LENGTH_EXTEND_FACTOR = value; }
    void setMinContourPoints(int value) { MIN_CONTOUR_POINTS = value; }
    void setMaxDeviationRatio(float value) { MAX_DEVIATION_RATIO = value; }
    void setMinLightbarRatio(float value) { MIN_LIGHTBAR_RATIO = value; }
    void setBrightnessPercentile(float value) { BRIGHTNESS_PERCENTILE = value; }
    void setColorWeight(float value) { COLOR_WEIGHT = value; }
    
    // 设置角点验证参数
    void setMinAspectRatio(float value) { MIN_ASPECT_RATIO = value; }
    void setMaxAspectRatio(float value) { MAX_ASPECT_RATIO = value; }
    void setParallelToleranceDeg(float value) { PARALLEL_TOLERANCE_DEG = value; }
    void setMinEdgeRatio(float value) { MIN_EDGE_RATIO = value; }
    void setMaxEdgeRatio(float value) { MAX_EDGE_RATIO = value; }
    void setMinDiagRatio(float value) { MIN_DIAG_RATIO = value; }
    void setMaxDiagRatio(float value) { MAX_DIAG_RATIO = value; }
    void setMinAreaRatio(float value) { MIN_AREA_RATIO = value; }
    void setMaxAreaRatio(float value) { MAX_AREA_RATIO = value; }
    
    // 获取角点优化参数
    float getROIExpandRatio() const { return ROI_EXPAND_RATIO; }
    float getLengthExtendFactor() const { return LENGTH_EXTEND_FACTOR; }
    int getMinContourPoints() const { return MIN_CONTOUR_POINTS; }
    float getMaxDeviationRatio() const { return MAX_DEVIATION_RATIO; }
    float getMinLightbarRatio() const { return MIN_LIGHTBAR_RATIO; }
    float getBrightnessPercentile() const { return BRIGHTNESS_PERCENTILE; }
    float getColorWeight() const { return COLOR_WEIGHT; }
    
    // 获取角点验证参数
    float getMinAspectRatio() const { return MIN_ASPECT_RATIO; }
    float getMaxAspectRatio() const { return MAX_ASPECT_RATIO; }
    float getParallelToleranceDeg() const { return PARALLEL_TOLERANCE_DEG; }
    float getMinEdgeRatio() const { return MIN_EDGE_RATIO; }
    float getMaxEdgeRatio() const { return MAX_EDGE_RATIO; }
    float getMinDiagRatio() const { return MIN_DIAG_RATIO; }
    float getMaxDiagRatio() const { return MAX_DIAG_RATIO; }
    float getMinAreaRatio() const { return MIN_AREA_RATIO; }
    float getMaxAreaRatio() const { return MAX_AREA_RATIO; }
    
    bool isCornerCorrectionEnabled() const { return enable_corner_correction; }
    bool isDebugModeEnabled() const { return debug_mode; }
    
    // 更新配置参数
    void updateConfig(ConfigManager* configManager) {
        if (!configManager) return;
        
        // 保存角点优化基本参数
        configManager->writeBool("corner_optimization", enable_corner_correction);
        configManager->writeDouble("ROI_EXPAND_RATIO", ROI_EXPAND_RATIO);
        configManager->writeDouble("LENGTH_EXTEND_FACTOR", LENGTH_EXTEND_FACTOR);
        configManager->writeInt("MIN_CONTOUR_POINTS", MIN_CONTOUR_POINTS);
        configManager->writeDouble("MAX_DEVIATION_RATIO", MAX_DEVIATION_RATIO);
        configManager->writeDouble("MIN_LIGHTBAR_RATIO", MIN_LIGHTBAR_RATIO);
        configManager->writeDouble("BRIGHTNESS_PERCENTILE", BRIGHTNESS_PERCENTILE);
        configManager->writeDouble("COLOR_WEIGHT", COLOR_WEIGHT);
        
        // 保存角点验证参数
        configManager->writeDouble("MIN_ASPECT_RATIO", MIN_ASPECT_RATIO);
        configManager->writeDouble("MAX_ASPECT_RATIO", MAX_ASPECT_RATIO);
        configManager->writeDouble("PARALLEL_TOLERANCE_DEG", PARALLEL_TOLERANCE_DEG);
        configManager->writeDouble("MIN_EDGE_RATIO", MIN_EDGE_RATIO);
        configManager->writeDouble("MAX_EDGE_RATIO", MAX_EDGE_RATIO);
        configManager->writeDouble("MIN_DIAG_RATIO", MIN_DIAG_RATIO);
        configManager->writeDouble("MAX_DIAG_RATIO", MAX_DIAG_RATIO);
        configManager->writeDouble("MIN_AREA_RATIO", MIN_AREA_RATIO);
        configManager->writeDouble("MAX_AREA_RATIO", MAX_AREA_RATIO);
        
        configManager->writeInt("debug_status", debug_mode ? 1 : 0);
        
        // 保存更新后的配置
        configManager->saveConfig();
    }
    
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
    
    // 启用或禁用角点优化
    void enableCornerCorrection(bool enable, bool debug = true) {
        enable_corner_correction = enable;
        debug_mode = debug;
        std::cout << "角点优化: " << (enable ? "已启用" : "已禁用") 
                  << " | 调试模式: " << (debug ? "已启用" : "已禁用") << std::endl;
    }
    
    // 处理图像并进行推理
    bool process(cv::Mat& image, std::vector<Armor>& armors, float conf_thr = 0.5, float iou_thr = 0.45, bool corner_correction_override = false) {
        try {
            // 图像预处理
            cv::Mat processed_image;
            preprocess_image(image, processed_image);
            
            // 准备输入张量
            ov::Tensor input_tensor = infer_request.get_input_tensor();
            preprocess(processed_image, input_tensor);
            
            // 执行推理
            auto infer_start = std::chrono::steady_clock::now();
            infer_request.infer();
            auto infer_end = std::chrono::steady_clock::now();
            // std::cout << "推理时间: " << std::chrono::duration<double>(infer_end - infer_start).count() * 1000 << " 毫秒" << std::endl;
            
            // 获取输出并处理
            ov::Tensor output_tensor = infer_request.get_output_tensor();
            auto result = output_tensor.data<float>();
            
            // 执行NMS获取装甲板检测结果
            nms(result, conf_thr, iou_thr, armors, 43);
            
            // 确定是否使用角点优化：使用参数覆盖或类设置
            bool use_corner_correction = corner_correction_override || enable_corner_correction;
            // std::cout << "角点优化状态: 参数覆盖=" << corner_correction_override 
            //           << ", 类设置=" << enable_corner_correction 
            //           << ", 最终使用=" << use_corner_correction << std::endl;
            // std::cout << "检测到装甲板数量: " << armors.size() << std::endl;
            
            // 如果启用角点优化，则对装甲板进行角点优化处理
            if (use_corner_correction && !armors.empty()) {
                std::cout << "检测到装甲板，进行角点优化..." << std::endl;
                auto corrected_armors = performCornerCorrection(image, armors);
                armors = corrected_armors;
            }
            
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
            // 如果是无效装甲板（角点优化失败的标记），则跳过绘制
            if (armor.x1 < 0 || armor.y1 < 0) {
                continue;
            }
            
            // 映射坐标回原始图像
            int x1 = int((armor.x1 - padding_x) / scale);
            int y1 = int((armor.y1 - padding_y) / scale);
            int x2 = int((armor.x2 - padding_x) / scale);
            int y2 = int((armor.y2 - padding_y) / scale);
            int x3 = int((armor.x3 - padding_x) / scale);
            int y3 = int((armor.y3 - padding_y) / scale);
            int x4 = int((armor.x4 - padding_x) / scale);
            int y4 = int((armor.y4 - padding_y) / scale);
            
            // 确保坐标在图像范围内
            x1 = std::max(0, std::min(x1, image.cols - 1));
            y1 = std::max(0, std::min(y1, image.rows - 1));
            x2 = std::max(0, std::min(x2, image.cols - 1));
            y2 = std::max(0, std::min(y2, image.rows - 1));
            x3 = std::max(0, std::min(x3, image.cols - 1));
            y3 = std::max(0, std::min(y3, image.rows - 1));
            x4 = std::max(0, std::min(x4, image.cols - 1));
            y4 = std::max(0, std::min(y4, image.rows - 1));
            
            // 绘制装甲板
            cv::Scalar color = COLORS[armor.label % COLORS.size()];
            
            // 根据类型选择不同线宽，蓝色和红色装甲板使用粗线
            int lineWidth = 2;
            if (armor.label % 3 == 0 || armor.label % 3 == 1) { // 蓝色或红色装甲板
                lineWidth = 3;
            }
            
            std::vector<cv::Point> polygon = {
                cv::Point(x1, y1), cv::Point(x2, y2), 
                cv::Point(x3, y3), cv::Point(x4, y4)
            };
            
            cv::polylines(visualization_image, std::vector<std::vector<cv::Point>>{polygon}, true, color, lineWidth);
            
            // 绘制角点
            if (enable_corner_correction) {
                // 绘制优化后的角点（使用较小的圆点）
                cv::circle(visualization_image, cv::Point(x1, y1), 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(visualization_image, cv::Point(x2, y2), 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(visualization_image, cv::Point(x3, y3), 3, cv::Scalar(0, 255, 0), -1);
                cv::circle(visualization_image, cv::Point(x4, y4), 3, cv::Scalar(0, 255, 0), -1);
                
                // 在角点旁添加优化标记
                cv::putText(visualization_image, "C", cv::Point(x1 - 10, y1 - 5),
                          cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);
            } else {
                // 传统方式绘制角点
                cv::circle(visualization_image, cv::Point(x1, y1), 5, cv::Scalar(0, 0, 255), -1);
                cv::circle(visualization_image, cv::Point(x2, y2), 5, cv::Scalar(0, 255, 0), -1);
                cv::circle(visualization_image, cv::Point(x3, y3), 5, cv::Scalar(255, 0, 0), -1);
                cv::circle(visualization_image, cv::Point(x4, y4), 5, cv::Scalar(255, 255, 0), -1);
            }
            
            // 获取简短标签名
            std::string shortLabel;
            if (armor.label < CLASS_NAMES.size()) {
                std::string fullName = CLASS_NAMES[armor.label];
                size_t lastUnderscore = fullName.find_last_of('_');
                
                if (lastUnderscore != std::string::npos && lastUnderscore + 1 < fullName.length()) {
                    // 获取最后一个下划线后的内容（颜色信息：blue/red/none）
                    std::string colorInfo = fullName.substr(lastUnderscore + 1);
                    
                    // 获取类型信息（从第一个下划线后到最后一个下划线前）
                    size_t firstUnderscore = fullName.find_first_of('_');
                    if (firstUnderscore != std::string::npos && firstUnderscore < lastUnderscore) {
                        std::string typeInfo = fullName.substr(firstUnderscore + 1, lastUnderscore - firstUnderscore - 1);
                        
                        // 创建简短标签：类型+颜色
                        shortLabel = typeInfo + "_" + colorInfo;
                    } else {
                        shortLabel = fullName;
                    }
                } else {
                    shortLabel = fullName;
                }
            } else {
                shortLabel = "class" + std::to_string(armor.label);
            }
            
            // 标签信息
            std::string label = shortLabel + " " + std::to_string(int(armor.score * 100)) + "%";
            
            cv::putText(visualization_image, label, cv::Point(x1, y1 - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
            
            std::cout << "装甲板: 类别=" << (armor.label < CLASS_NAMES.size() ? CLASS_NAMES[armor.label] : "未知") 
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
        cv::copyMakeBorder(processed_image, processed_image, padding_y, padding_y, padding_x, padding_x, 
                          cv::BORDER_CONSTANT, cv::Scalar(144, 144, 144));
    }
    
    // 装甲板角点优化处理
    std::vector<Armor> performCornerCorrection(cv::Mat& image, const std::vector<Armor>& armors) {
        std::vector<Armor> corrected_armors;
        
        // 先将处理后的图像转换回原始尺寸，以便进行角点优化
        cv::Mat original_scale_image = image.clone();
        
        for (const auto& armor : armors) {
            // 创建ArmorExt实例来进行角点优化
            ArmorExt armor_ext(armor);
            
            // 映射坐标回原始图像尺寸，因为角点优化在原始图像上进行
            armor_ext.x[0] = (armor.x1 - padding_x) / scale;
            armor_ext.y[0] = (armor.y1 - padding_y) / scale;
            armor_ext.x[1] = (armor.x2 - padding_x) / scale;
            armor_ext.y[1] = (armor.y2 - padding_y) / scale;
            armor_ext.x[2] = (armor.x3 - padding_x) / scale;
            armor_ext.y[2] = (armor.y3 - padding_y) / scale;
            armor_ext.x[3] = (armor.x4 - padding_x) / scale;
            armor_ext.y[3] = (armor.y4 - padding_y) / scale;
            armor_ext.score = armor.score;
            armor_ext.label = armor.label;
            
            // 使用从配置文件加载的所有参数执行角点优化（包括验证参数）
            bool correction_success = armor_ext.correct_promote(
                original_scale_image,
                ROI_EXPAND_RATIO,
                LENGTH_EXTEND_FACTOR,
                MIN_CONTOUR_POINTS,
                MAX_DEVIATION_RATIO,
                MIN_LIGHTBAR_RATIO,
                BRIGHTNESS_PERCENTILE,
                COLOR_WEIGHT,
                debug_mode,
                // 添加角点验证参数
                MIN_ASPECT_RATIO,
                MAX_ASPECT_RATIO,
                PARALLEL_TOLERANCE_DEG,
                MIN_EDGE_RATIO,
                MAX_EDGE_RATIO,
                MIN_DIAG_RATIO,
                MAX_DIAG_RATIO,
                MIN_AREA_RATIO,
                MAX_AREA_RATIO
            );
            
            // 如果角点优化成功，则映射回处理后的坐标
            if (correction_success) {
                std::cout << "角点优化成功" << std::endl;
                // 映射坐标回处理后的图像坐标
                for (int i = 0; i < 4; ++i) {
                    armor_ext.x[i] = armor_ext.x[i] * scale + padding_x;
                    armor_ext.y[i] = armor_ext.y[i] * scale + padding_y;
                }
                
                // 将优化后的装甲板转换回基础Armor结构并添加到结果集
                corrected_armors.push_back(armor_ext.toArmor());
            } else {
                // 角点优化失败，设置特殊标记或使用原始值
                std::cout << "角点优化失败" << std::endl;
                Armor failed_armor = armor;
                failed_armor.x1 = -1; // 使用负值作为失败标记
                failed_armor.y1 = -1;
                corrected_armors.push_back(failed_armor);
            }
        }
        
        return corrected_armors;
    }
};

#endif // ARMOR_DETECT_HPP