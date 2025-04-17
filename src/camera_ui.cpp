#include "CameraUI.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <ArmorDetect.hpp>

// 构造函数
CameraUI::CameraUI(SimpleCamera& cam, const std::string& name)
    : camera(cam), windowName(name), running(false), armorDetector(nullptr) {
}

// 析构函数
CameraUI::~CameraUI() {
    stop();
}

// 初始化UI
void CameraUI::init() {
    try {
        std::cout << "正在初始化相机UI界面..." << std::endl;
        
        // 在创建窗口前设置全局属性（这些是自定义环境变量，不是函数调用）
        #if CV_MAJOR_VERSION >= 4
        // OpenCV 4+的一些特殊设置
        // 注意：这些不是直接通过函数调用设置的，而是通过环境变量
        // setenv("OPENCV_WINDOW_NO_TITLEBAR", "0", 1);  // 保留标题栏
        // setenv("OPENCV_WINDOW_FREERATIO", "0", 1);    // 不使用自由比例
        #endif
        
        // 创建用于显示相机图像的窗口 - 使用更简单的WINDOW_NORMAL
        cv::namedWindow(windowName, cv::WINDOW_NORMAL);
        cv::resizeWindow(windowName, 800, 600);
        
        // 设置图像窗口背景颜色（尝试解决黑框问题）
        cv::Mat background(600, 800, CV_8UC3, cv::Scalar(240, 240, 240));
        cv::imshow(windowName, background);
        cv::waitKey(1);  // 刷新显示
        
        // 创建控制相机参数的窗口
        cv::namedWindow("Controls", cv::WINDOW_NORMAL);
        cv::resizeWindow("Controls", 600, 400);
        
        // 设置窗口属性（使用正确的API）
        cv::setWindowProperty(windowName, cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
        cv::setWindowProperty("Controls", cv::WND_PROP_ASPECT_RATIO, cv::WINDOW_KEEPRATIO);
        
        // 强制窗口刷新并等待
        cv::waitKey(200); // 给窗口管理器更多时间来处理窗口创建
        
        // 创建所有滑动条
        createTrackbars();
        
        // 创建按钮控件
        createButtons();
        
        // 从相机读取参数更新界面
        updateTrackbarsFromCamera();
        
        std::cout << "相机UI界面初始化完成" << std::endl;
        
        // 再次显示背景并强制刷新
        cv::imshow(windowName, background);
        cv::waitKey(1);
    }
    catch (const cv::Exception& e) {
        std::cerr << "创建UI界面时发生OpenCV异常: " << e.what() << std::endl;
        throw; // 重新抛出异常，让主程序处理
    }
}

// 从相机参数更新UI界面
void CameraUI::updateTrackbarsFromCamera() {
    // 更新滑动条的位置，根据相机当前参数
    cv::setTrackbarPos("曝光时间", "Controls", camera.getExposureTime());
    cv::setTrackbarPos("增益", "Controls", camera.getAnalogGain());
    cv::setTrackbarPos("亮度", "Controls", camera.getBrightness());
    cv::setTrackbarPos("Gamma", "Controls", camera.getGamma());
    cv::setTrackbarPos("对比度", "Controls", camera.getContrast() + 50); // 对比度有-50的偏移
    cv::setTrackbarPos("饱和度", "Controls", camera.getSaturation());
    cv::setTrackbarPos("锐度", "Controls", camera.getSharpness());
    cv::setTrackbarPos("R增益", "Controls", camera.getRGain());
    cv::setTrackbarPos("G增益", "Controls", camera.getGGain());
    cv::setTrackbarPos("B增益", "Controls", camera.getBGain());
    cv::setTrackbarPos("自动曝光", "Controls", camera.getAutoExposure() ? 1 : 0);
    cv::setTrackbarPos("自动白平衡", "Controls", camera.getAutoWhiteBalance() ? 1 : 0);
    
    // 更新角点优化参数滑动条
    ConfigManager* config = camera.getConfigManager();
    if(config) {
        bool corner_opt = false;
        if(config->readBool("corner_optimization", corner_opt)) {
            cv::setTrackbarPos("启用角点优化", "Corner Optimization", corner_opt ? 1 : 0);
        }
        
        double value;
        if(config->readDouble("ROI_EXPAND_RATIO", value)) {
            cv::setTrackbarPos("ROI扩展比例", "Corner Optimization", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("LENGTH_EXTEND_FACTOR", value)) {
            cv::setTrackbarPos("灯条扩展系数", "Corner Optimization", static_cast<int>(value * 10));
        }
        
        int int_value;
        if(config->readInt("MIN_CONTOUR_POINTS", int_value)) {
            cv::setTrackbarPos("最小轮廓点数", "Corner Optimization", int_value);
        }
        
        if(config->readDouble("MAX_DEVIATION_RATIO", value)) {
            cv::setTrackbarPos("最大偏差比例", "Corner Optimization", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("MIN_LIGHTBAR_RATIO", value)) {
            cv::setTrackbarPos("最小灯条比例", "Corner Optimization", static_cast<int>(value * 10));
        }
        
        if(config->readDouble("BRIGHTNESS_PERCENTILE", value)) {
            cv::setTrackbarPos("亮度百分位阈值", "Corner Optimization", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("COLOR_WEIGHT", value)) {
            cv::setTrackbarPos("颜色权重", "Corner Optimization", static_cast<int>(value * 100));
        }
        
        if(config->readInt("debug_status", int_value)) {
            cv::setTrackbarPos("调试模式", "Corner Optimization", int_value);
        }
        
        // 添加角点验证参数滑动条初始值设置
        if(config->readDouble("MIN_ASPECT_RATIO", value)) {
            cv::setTrackbarPos("最小宽高比*10", "Corner Validation", static_cast<int>(value * 10));
        }
        
        if(config->readDouble("MAX_ASPECT_RATIO", value)) {
            cv::setTrackbarPos("最大宽高比*10", "Corner Validation", static_cast<int>(value * 10));
        }
        
        if(config->readDouble("PARALLEL_TOLERANCE_DEG", value)) {
            cv::setTrackbarPos("平行度容差(度)", "Corner Validation", static_cast<int>(value));
        }
        
        if(config->readDouble("MIN_EDGE_RATIO", value)) {
            cv::setTrackbarPos("最小对边比*100", "Corner Validation", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("MAX_EDGE_RATIO", value)) {
            cv::setTrackbarPos("最大对边比*100", "Corner Validation", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("MIN_DIAG_RATIO", value)) {
            cv::setTrackbarPos("最小对角线比*100", "Corner Validation", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("MAX_DIAG_RATIO", value)) {
            cv::setTrackbarPos("最大对角线比*100", "Corner Validation", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("MIN_AREA_RATIO", value)) {
            cv::setTrackbarPos("最小面积比*100", "Corner Validation", static_cast<int>(value * 100));
        }
        
        if(config->readDouble("MAX_AREA_RATIO", value)) {
            cv::setTrackbarPos("最大面积比*100", "Corner Validation", static_cast<int>(value * 100));
        }
    }
}

// 创建所有滑动条
void CameraUI::createTrackbars() {
    // 曝光时间 (0-20000微秒)
    cv::createTrackbar("曝光时间", "Controls", nullptr, 20000, onExposureChange, this);
    
    // 模拟增益 (1-16)
    cv::createTrackbar("增益", "Controls", nullptr, 16, onGainChange, this);
    
    // 亮度 (0-1000)
    cv::createTrackbar("亮度", "Controls", nullptr, 1000, onBrightnessChange, this);
    
    // Gamma (0-350)
    cv::createTrackbar("Gamma", "Controls", nullptr, 350, onGammaChange, this);
    
    // 对比度 (-50-100)
    cv::createTrackbar("对比度", "Controls", nullptr, 150, onContrastChange, this);
    
    // 饱和度 (0-200)
    cv::createTrackbar("饱和度", "Controls", nullptr, 200, onSaturationChange, this);
    
    // 锐度 (0-100)
    cv::createTrackbar("锐度", "Controls", nullptr, 100, onSharpnessChange, this);
    
    // R增益 (0-400)
    cv::createTrackbar("R增益", "Controls", nullptr, 400, onRGainChange, this);
    
    // G增益 (0-400)
    cv::createTrackbar("G增益", "Controls", nullptr, 400, onGGainChange, this);
    
    // B增益 (0-400)
    cv::createTrackbar("B增益", "Controls", nullptr, 400, onBGainChange, this);
    
    // 自动曝光 (0-1)
    cv::createTrackbar("自动曝光", "Controls", nullptr, 1, onAutoExposureChange, this);
    
    // 自动白平衡 (0-1)
    cv::createTrackbar("自动白平衡", "Controls", nullptr, 1, onAutoWBChange, this);
    
    // 创建角点优化参数控制器
    cv::namedWindow("Corner Optimization", cv::WINDOW_NORMAL);
    cv::resizeWindow("Corner Optimization", 600, 400);
    
    // 角点优化开关 (0-1)
    cv::createTrackbar("启用角点优化", "Corner Optimization", nullptr, 1, onCornerOptimizationChange, this);
    
    // ROI扩展比例 (1-100，实际值为0.1-1.0)
    cv::createTrackbar("ROI扩展比例", "Corner Optimization", nullptr, 100, onROIExpandRatioChange, this);
    
    // 灯条长度扩展系数 (10-30，实际值为1.0-3.0)
    cv::createTrackbar("灯条扩展系数", "Corner Optimization", nullptr, 30, onLengthExtendFactorChange, this);
    
    // 最小轮廓点数 (3-20)
    cv::createTrackbar("最小轮廓点数", "Corner Optimization", nullptr, 20, onMinContourPointsChange, this);
    
    // 最大偏差比例 (50-100，实际值为0.5-1.0)
    cv::createTrackbar("最大偏差比例", "Corner Optimization", nullptr, 100, onMaxDeviationRatioChange, this);
    
    // 最小灯条比例 (10-50，实际值为1.0-5.0)
    cv::createTrackbar("最小灯条比例", "Corner Optimization", nullptr, 50, onMinLightbarRatioChange, this);
    
    // 亮度百分位阈值 (1-20，实际值为0.01-0.2)
    cv::createTrackbar("亮度百分位阈值", "Corner Optimization", nullptr, 20, onBrightnessPercentileChange, this);
    
    // 颜色权重 (0-100，实际值为0.0-1.0)
    cv::createTrackbar("颜色权重", "Corner Optimization", nullptr, 100, onColorWeightChange, this);
    
    // 调试模式开关 (0-1)
    cv::createTrackbar("调试模式", "Corner Optimization", nullptr, 1, onDebugModeChange, this);

    // 创建角点验证参数控制器
    cv::namedWindow("Corner Validation", cv::WINDOW_NORMAL);
    cv::resizeWindow("Corner Validation", 600, 400);
    
    // 宽高比范围 (10-50，实际值为1.0-5.0)
    cv::createTrackbar("最小宽高比*10", "Corner Validation", nullptr, 50, onMinAspectRatioChange, this);
    cv::createTrackbar("最大宽高比*10", "Corner Validation", nullptr, 100, onMaxAspectRatioChange, this);
    
    // 平行度容差 (1-20，实际值为1.0-20.0度)
    cv::createTrackbar("平行度容差(度)", "Corner Validation", nullptr, 20, onParallelToleranceDegChange, this);
    
    // 对边长度比范围 (50-120，实际值为0.5-1.2)
    cv::createTrackbar("最小对边比*100", "Corner Validation", nullptr, 100, onMinEdgeRatioChange, this);
    cv::createTrackbar("最大对边比*100", "Corner Validation", nullptr, 200, onMaxEdgeRatioChange, this);
    
    // 对角线比范围 (50-120，实际值为0.5-1.2)
    cv::createTrackbar("最小对角线比*100", "Corner Validation", nullptr, 100, onMinDiagRatioChange, this);
    cv::createTrackbar("最大对角线比*100", "Corner Validation", nullptr, 200, onMaxDiagRatioChange, this);
    
    // 面积比范围 (50-150，实际值为0.5-1.5)
    cv::createTrackbar("最小面积比*100", "Corner Validation", nullptr, 100, onMinAreaRatioChange, this);
    cv::createTrackbar("最大面积比*100", "Corner Validation", nullptr, 200, onMaxAreaRatioChange, this);
}

// 创建按钮控件
void CameraUI::createButtons() {
    // 创建一个保存配置的按钮（使用跟踪条模拟按钮）
    cv::createTrackbar("保存配置", "Controls", nullptr, 1, onSaveConfigButtonClick, this);
    cv::setTrackbarPos("保存配置", "Controls", 0);
}

// 运行UI主循环
bool CameraUI::run() {
    running = true;
    cv::Mat frame;
    
    std::cout << "UI主循环开始运行..." << std::endl;
    
    // 创建背景图像
    cv::Mat background(600, 800, CV_8UC3, cv::Scalar(240, 240, 240));
    cv::putText(background, "等待相机图像...", cv::Point(250, 300), 
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 2);
    cv::imshow(windowName, background);
    cv::waitKey(1);  // 刷新显示
    
    // 创建并初始化装甲板检测器
    std::string model_path = std::filesystem::absolute("../model/last.xml").string();
    ArmorDetector detector(model_path, camera.getConfigManager());
    armorDetector = &detector; // 设置指针，使回调函数可以访问这个实例
    bool detector_initialized = false;
    
    if (std::filesystem::exists(model_path)) {
        detector_initialized = armorDetector->init();
        if (detector_initialized) {
            std::cout << "装甲板检测器初始化成功" << std::endl;
            
            // 从配置中加载角点优化参数
            armorDetector->loadConfig(camera.getConfigManager());
            
            // 显示当前角点优化状态
            bool corner_opt = armorDetector->isCornerCorrectionEnabled();
            bool debug_mode = armorDetector->isDebugModeEnabled();
            
            std::cout << "角点优化状态: " << (corner_opt ? "开启" : "关闭") 
                      << ", 调试模式: " << (debug_mode ? "开启" : "关闭") << std::endl;
            
            // 显式打印角点优化参数
            std::cout << "角点优化参数: "
                      << "ROI扩展比例=" << armorDetector->getROIExpandRatio()
                      << ", 灯条扩展系数=" << armorDetector->getLengthExtendFactor()
                      << ", 最小轮廓点数=" << armorDetector->getMinContourPoints()
                      << ", 最大偏差比例=" << armorDetector->getMaxDeviationRatio()
                      << ", 最小灯条比例=" << armorDetector->getMinLightbarRatio()
                      << ", 亮度百分位阈值=" << armorDetector->getBrightnessPercentile()
                      << ", 颜色权重=" << armorDetector->getColorWeight()
                      << std::endl;
        } else {
            std::cerr << "装甲板检测器初始化失败" << std::endl;
        }
    } else {
        std::cerr << "模型文件不存在: " << model_path << std::endl;
    }
    
    while (running) {
        try {
            // 获取相机图像
            if (camera.getFrame(frame)) {
                if (!frame.empty()) {
                    // 确保图像格式正确（BGR格式用于显示）
                    cv::Mat displayFrame;
                    if (frame.channels() == 1) {
                        // 如果是灰度图，转换为BGR
                        cv::cvtColor(frame, displayFrame, cv::COLOR_GRAY2BGR);
                    } else {
                        displayFrame = frame.clone();
                    }
                    
                    // 调整图像大小以适应窗口（避免出现黑边）
                    cv::resize(displayFrame, displayFrame, cv::Size(800, 600), 0, 0, cv::INTER_AREA);
                    
                    // 确保图像边界完整（添加边框以防止黑边）
                    cv::copyMakeBorder(displayFrame, displayFrame, 1, 1, 1, 1, 
                                      cv::BORDER_CONSTANT, cv::Scalar(240, 240, 240));

                    // 装甲板检测部分
                    if (detector_initialized && armorDetector) {
                        std::vector<Armor> armors;
                        
                        // 每帧重新从检测器实例获取参数状态，不从配置文件读取
                        bool corner_opt = armorDetector->isCornerCorrectionEnabled();
                        bool debug_mode = armorDetector->isDebugModeEnabled();
                        
                        // 处理图像，使用当前的角点优化状态
                        if (armorDetector->process(displayFrame, armors, 0.5, 0.45)) {
                            // 可视化结果
                            armorDetector->visualize(displayFrame, armors);
                        }
                    }
                    
                    // 显示图像
                    cv::imshow(windowName, displayFrame);
                } else {
                    std::cerr << "获取到空图像" << std::endl;
                    // 显示背景，防止黑屏
                    cv::imshow(windowName, background);
                }
            } else {
                std::cerr << "获取图像失败" << std::endl;
                // 显示背景，防止黑屏
                cv::imshow(windowName, background);
            }
            
            // 处理键盘输入
            int key = cv::waitKey(10);
            if (key == 27) { // ESC键
                std::cout << "检测到ESC键，退出UI循环" << std::endl;
                running = false;
            } else if (key == 's') { // 's'键保存图像
                if (!frame.empty()) {
                    std::string filename = "camera_capture_" + 
                        std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".jpg";
                    cv::imwrite(filename, frame);
                    std::cout << "图片已保存为：" << filename << std::endl;
                }
            } else if (key == 'c') { // 'c'键保存配置
                camera.saveParametersToConfig();
                if (armorDetector) {
                    armorDetector->updateConfig(camera.getConfigManager());
                }
                std::cout << "配置已保存" << std::endl;
            } else if (key == 'r') { // 'r'键刷新窗口
                // 重新调整窗口大小，可能有助于解决显示问题
                cv::resizeWindow(windowName, 800, 600);
                cv::resizeWindow("Corner Optimization", 600, 400); // 同时调整角点优化窗口大小
                if (!frame.empty()) {
                    cv::imshow(windowName, frame);
                } else {
                    cv::imshow(windowName, background);
                }
                cv::waitKey(1);
            } else if (key == 'o') { // 'o'键切换角点优化
                if (armorDetector) {
                    bool corner_opt = !armorDetector->isCornerCorrectionEnabled();
                    armorDetector->enableCornerCorrection(corner_opt);
                    cv::setTrackbarPos("启用角点优化", "Corner Optimization", corner_opt ? 1 : 0);
                    camera.getConfigManager()->writeBool("corner_optimization", corner_opt);
                    // camera.getConfigManager()->saveConfig();
                    std::cout << "角点优化: " << (corner_opt ? "开启" : "关闭") << std::endl;
                }
            } else if (key == 'd') { // 'd'键切换调试模式
                if (armorDetector) {
                    bool debug_mode = !armorDetector->isDebugModeEnabled();
                    armorDetector->enableCornerCorrection(armorDetector->isCornerCorrectionEnabled(), debug_mode);
                    cv::setTrackbarPos("调试模式", "Corner Optimization", debug_mode ? 1 : 0);
                    camera.getConfigManager()->writeInt("debug_status", debug_mode ? 1 : 0);
                    // camera.getConfigManager()->saveConfig();
                    std::cout << "调试模式: " << (debug_mode ? "开启" : "关闭") << std::endl;
                }
            }
        }
        catch (const cv::Exception& e) {
            std::cerr << "UI循环中发生OpenCV异常: " << e.what() << std::endl;
            std::cerr << "尝试恢复..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    armorDetector = nullptr; // 清除指针，防止悬空引用
    return true;
}

// 停止UI
void CameraUI::stop() {
    running = false;
    cv::destroyWindow(windowName);
    cv::destroyWindow("Controls");
    cv::destroyWindow("Corner Optimization"); // 添加销毁角点优化窗口
    cv::destroyWindow("Corner Validation"); // 添加销毁角点验证窗口
    cv::destroyAllWindows(); // 确保关闭所有可能的调试窗口
}

// 回调函数实现
void CameraUI::onExposureChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setExposureTime(static_cast<double>(pos));
    std::cout << "曝光时间设置为: " << pos << "微秒" << std::endl;
}

void CameraUI::onGainChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAnalogGain(static_cast<double>(pos));
    std::cout << "增益设置为: " << pos << std::endl;
}

void CameraUI::onBrightnessChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setBrightness(pos);
    std::cout << "亮度设置为: " << pos << std::endl;
}

void CameraUI::onGammaChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setGamma(pos);
    std::cout << "Gamma设置为: " << pos << std::endl;
}

void CameraUI::onContrastChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 对比度范围是-50到100，但滑动条范围是0到150
    ui->camera.setContrast(pos - 50);
    std::cout << "对比度设置为: " << (pos - 50) << std::endl;
}

void CameraUI::onSaturationChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setSaturation(pos);
    std::cout << "饱和度设置为: " << pos << std::endl;
}

void CameraUI::onSharpnessChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setSharpness(pos);
    std::cout << "锐度设置为: " << pos << std::endl;
}

void CameraUI::onRGainChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setRGain(pos);
    std::cout << "R增益设置为: " << pos << std::endl;
}

void CameraUI::onGGainChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setGGain(pos);
    std::cout << "G增益设置为: " << pos << std::endl;
}

void CameraUI::onBGainChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setBGain(pos);
    std::cout << "B增益设置为: " << pos << std::endl;
}

void CameraUI::onAutoExposureChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAutoExposure(pos == 1);
    std::cout << "自动曝光: " << (pos == 1 ? "开启" : "关闭") << std::endl;
}

void CameraUI::onAutoWBChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAutoWhiteBalance(pos == 1);
    std::cout << "自动白平衡: " << (pos == 1 ? "开启" : "关闭") << std::endl;
}

void CameraUI::onSaveConfigButtonClick(int state, void* userdata) {
    if (state == 1) {
        CameraUI* ui = static_cast<CameraUI*>(userdata);
        // 保存配置
        ui->camera.saveParametersToConfig();
        std::cout << "相机参数已保存到配置文件" << std::endl;
        // 将按钮状态恢复为0
        cv::setTrackbarPos("保存配置", "Controls", 0);
    }
}

// 角点优化参数回调函数实现
void CameraUI::onCornerOptimizationChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->enableCornerCorrection(pos == 1);
        
        // 同时更新配置文件
        ui->camera.getConfigManager()->writeBool("corner_optimization", pos == 1);
        // ui->camera.getConfigManager()->saveConfig();
        
        std::cout << "角点优化: " << (pos == 1 ? "开启" : "关闭") << std::endl;
    }
}

void CameraUI::onROIExpandRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(1-100)转换为实际值(0.1-1.0)
    float value = pos / 100.0f;
    if (value < 0.1f) value = 0.1f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setROIExpandRatio(value);
    }
    
    // 更新配置，但不保存
    ui->camera.getConfigManager()->writeDouble("ROI_EXPAND_RATIO", value);
    // 删除自动保存的代码
    
    std::cout << "ROI扩展比例: " << value << std::endl;
}

void CameraUI::onLengthExtendFactorChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(10-30)转换为实际值(1.0-3.0)
    float value = pos / 10.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setLengthExtendFactor(value);
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeDouble("LENGTH_EXTEND_FACTOR", value);
    // 移除自动保存配置的代码
    
    std::cout << "灯条长度扩展系数: " << value << std::endl;
}

void CameraUI::onMinContourPointsChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 确保值至少为3
    int value = (pos < 3) ? 3 : pos;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMinContourPoints(value);
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeInt("MIN_CONTOUR_POINTS", value);
    // 移除自动保存配置的代码
    
    std::cout << "最小轮廓点数: " << value << std::endl;
}

void CameraUI::onMaxDeviationRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(50-100)转换为实际值(0.5-1.0)
    float value = pos / 100.0f;
    if (value < 0.5f) value = 0.5f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMaxDeviationRatio(value);
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeDouble("MAX_DEVIATION_RATIO", value);
    // 移除自动保存配置的代码
    
    std::cout << "最大偏差比例: " << value << std::endl;
}

void CameraUI::onMinLightbarRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(10-50)转换为实际值(1.0-5.0)
    float value = pos / 10.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMinLightbarRatio(value);
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeDouble("MIN_LIGHTBAR_RATIO", value);
    // 移除自动保存配置的代码
    
    std::cout << "最小灯条比例: " << value << std::endl;
}

void CameraUI::onBrightnessPercentileChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(1-20)转换为实际值(0.01-0.2)
    float value = pos / 100.0f;
    if (value < 0.01f) value = 0.01f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setBrightnessPercentile(value);
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeDouble("BRIGHTNESS_PERCENTILE", value);
    // 移除自动保存配置的代码
    
    std::cout << "亮度百分位阈值: " << value << std::endl;
}

void CameraUI::onColorWeightChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(0-100)转换为实际值(0.0-1.0)
    float value = pos / 100.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setColorWeight(value);
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeDouble("COLOR_WEIGHT", value);
    // 移除自动保存配置的代码
    
    std::cout << "颜色权重: " << value << std::endl;
}

void CameraUI::onDebugModeChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    
    if (ui->armorDetector) {
        // 直接更新实例参数，保持角点优化的开启/关闭状态不变
        ui->armorDetector->enableCornerCorrection(
            ui->armorDetector->isCornerCorrectionEnabled(), 
            pos == 1
        );
    }
    
    // 更新配置但不保存
    ui->camera.getConfigManager()->writeInt("debug_status", pos);
    // 移除自动保存配置的代码
    
    std::cout << "调试模式: " << (pos == 1 ? "开启" : "关闭") << std::endl;
}

// 角点验证参数回调函数实现
void CameraUI::onMinAspectRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(10-50)转换为实际值(1.0-5.0)
    float value = pos / 10.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMinAspectRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MIN_ASPECT_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最小宽高比: " << value << std::endl;
}

void CameraUI::onMaxAspectRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(10-100)转换为实际值(1.0-10.0)
    float value = pos / 10.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMaxAspectRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MAX_ASPECT_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最大宽高比: " << value << std::endl;
}

void CameraUI::onParallelToleranceDegChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(1-20)转换为实际角度(1.0-20.0度)
    float value = static_cast<float>(pos);
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setParallelToleranceDeg(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("PARALLEL_TOLERANCE_DEG", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "平行度容差: " << value << "度" << std::endl;
}

void CameraUI::onMinEdgeRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(50-100)转换为实际值(0.5-1.0)
    float value = pos / 100.0f;
    if (value < 0.5f) value = 0.5f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMinEdgeRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MIN_EDGE_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最小对边比: " << value << std::endl;
}

void CameraUI::onMaxEdgeRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(100-200)转换为实际值(1.0-2.0)
    float value = pos / 100.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMaxEdgeRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MAX_EDGE_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最大对边比: " << value << std::endl;
}

void CameraUI::onMinDiagRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(50-100)转换为实际值(0.5-1.0)
    float value = pos / 100.0f;
    if (value < 0.5f) value = 0.5f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMinDiagRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MIN_DIAG_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最小对角线比: " << value << std::endl;
}

void CameraUI::onMaxDiagRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(100-200)转换为实际值(1.0-2.0)
    float value = pos / 100.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMaxDiagRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MAX_DIAG_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最大对角线比: " << value << std::endl;
}

void CameraUI::onMinAreaRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(50-100)转换为实际值(0.5-1.0)
    float value = pos / 100.0f;
    if (value < 0.5f) value = 0.5f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMinAreaRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MIN_AREA_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最小面积比: " << value << std::endl;
}

void CameraUI::onMaxAreaRatioChange(int pos, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 将滑动条值(100-200)转换为实际值(1.0-2.0)
    float value = pos / 100.0f;
    if (value < 1.0f) value = 1.0f;
    
    if (ui->armorDetector) {
        // 直接更新实例参数
        ui->armorDetector->setMaxAreaRatio(value);
    }
    
    // 更新配置
    ui->camera.getConfigManager()->writeDouble("MAX_AREA_RATIO", value);
    ui->camera.getConfigManager()->saveConfig();
    
    std::cout << "最大面积比: " << value << std::endl;
}