#include "CameraUI.h"
#include <iostream>
#include <chrono>
#include <thread>
#include <ArmorDetect.hpp>
#include <ctime>
#include <iomanip> // 用于设置小数点精度

// 构造函数
CameraUI::CameraUI(SimpleCamera& cam, const std::string& name)
    : camera(cam), windowName(name), running(false) {
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
}

// 创建所有滑动条
void CameraUI::createTrackbars() {
    cv::createTrackbar("曝光值", windowName, &params.exposure, 20000, onExposureChange, this);
    cv::createTrackbar("增益", windowName, &params.gain, 100, onGainChange, this);
    cv::createTrackbar("伽马", windowName, &params.gamma, 200, onGammaChange, this); // Gamma typically 0.1 to 2.0, scale by 100
    cv::createTrackbar("对比度", windowName, &params.contrast, 200, onContrastChange, this); // Contrast typically 0 to 200
    cv::createTrackbar("饱和度", windowName, &params.saturation, 200, onSaturationChange, this); // Saturation typically 0 to 200
    cv::createTrackbar("锐度", windowName, &params.sharpness, 100, onSharpnessChange, this); // Sharpness typically 0 to 100
    cv::createTrackbar("白平衡红", windowName, &params.wbR, 400, onWBRChange, this); // WB typically 0 to 400
    cv::createTrackbar("白平衡绿", windowName, &params.wbG, 400, onWBGChange, this);
    cv::createTrackbar("白平衡蓝", windowName, &params.wbB, 400, onWBBChange, this);

    // Add new trackbars for armor detection thresholds
    // Assuming params will have these members, and they are int.
    // Max values are set to 255 as they seem to be pixel intensity thresholds.
    // Default values are from the user prompt.
    // TODO: Ensure these params are added to the CameraParams struct and initialized in loadConfig.
    cv::createTrackbar("Armor Brightness Thresh Red", windowName, nullptr, 255, onArmorBrightnessThresholdRedChange, this);
    cv::setTrackbarPos("Armor Brightness Thresh Red", windowName, 90);
    cv::createTrackbar("Armor Brightness Thresh Blue", windowName, nullptr, 255, onArmorBrightnessThresholdBlueChange, this);
    cv::setTrackbarPos("Armor Brightness Thresh Blue", windowName, 110);
    cv::createTrackbar("Brightness Thresh", windowName, nullptr, 255, onBrightnessThresholdChange, this);
    cv::setTrackbarPos("Brightness Thresh", windowName, 120);
    cv::createTrackbar("Channal Sum Thresh", windowName, nullptr, 255, onChannalSumThresholdChange, this);
    cv::setTrackbarPos("Channal Sum Thresh", windowName, 150);
}

// 创建按钮控件
void CameraUI::createButtons() {

    // 自动曝光 (使用两个按钮替代拖动条)
    cv::createButton("自动曝光开", onAutoExposureOn, this, cv::QT_PUSH_BUTTON);
    cv::createButton("自动曝光关", onAutoExposureOff, this, cv::QT_PUSH_BUTTON);
    
    // 自动白平衡 (使用两个按钮替代拖动条)
    cv::createButton("自动白平衡开", onAutoWBOn, this, cv::QT_PUSH_BUTTON);
    cv::createButton("自动白平衡关", onAutoWBOff, this, cv::QT_PUSH_BUTTON);

    // 创建一个保存配置的按钮
    cv::createButton("保存配置", onSaveConfigButtonClick, this, cv::QT_PUSH_BUTTON);
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

    auto last_time = std::chrono::high_resolution_clock::now();
    int frame_count = 0;
    double fps = 0.0;

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
                    std::string model_path = std::filesystem::absolute("../model/last.xml").string();
                    if (std::filesystem::exists(model_path)) {
                        static ArmorDetector armorDetector(model_path);
                        static bool detector_initialized = false;
                        
                        if (!detector_initialized) {
                            detector_initialized = armorDetector.init();
                            if (!detector_initialized) {
                                std::cerr << "装甲板检测器初始化失败" << std::endl;
                            }
                        }
                        
                        if (detector_initialized) {
                            std::vector<Armor> armors;
                            
                            // 修改帧率计算逻辑，直接计算每次推理的耗费时间
                            auto start_time = std::chrono::high_resolution_clock::now();

                            // 推理代码部分
                            if (armorDetector.process(displayFrame, armors)) {
                                // 可视化结果
                                armorDetector.visualize(displayFrame, armors);
                            }

                            // 计算推理耗时
                            auto end_time = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double, std::milli> inference_time = end_time - start_time;

                            fps = 1000.0 / inference_time.count();

                            // 在图像上显示帧率
                            std::ostringstream fps_text;
                            fps_text << std::fixed << std::setprecision(1) << "FPS: " << fps;
                            cv::putText(displayFrame, fps_text.str(), cv::Point(10, 30), 
                                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

                            // 在图像上显示推理耗时
                            std::ostringstream inference_text;
                            inference_text << "Inference: " << inference_time.count() << " ms";
                            cv::putText(displayFrame, inference_text.str(), cv::Point(10, 60), 
                                        cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
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
                std::cout << "配置已保存" << std::endl;
            } else if (key == 'r') { // 'r'键刷新窗口
                // 重新调整窗口大小，可能有助于解决显示问题
                cv::resizeWindow(windowName, 800, 600);
                if (!frame.empty()) {
                    cv::imshow(windowName, frame);
                } else {
                    cv::imshow(windowName, background);
                }
                cv::waitKey(1);
            }
        }
        catch (const cv::Exception& e) {
            std::cerr << "UI循环中发生OpenCV异常: " << e.what() << std::endl;
            std::cerr << "尝试恢复..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }
    
    return true;
}

// 停止UI
void CameraUI::stop() {
    running = false;
    cv::destroyWindow(windowName);
    cv::destroyWindow("Controls");
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

// 自动曝光开启按钮回调
void CameraUI::onAutoExposureOn(int, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAutoExposure(true);
    std::cout << "自动曝光: 开启" << std::endl;
}

// 自动曝光关闭按钮回调
void CameraUI::onAutoExposureOff(int, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAutoExposure(false);
    std::cout << "自动曝光: 关闭" << std::endl;
}

// 自动白平衡开启按钮回调
void CameraUI::onAutoWBOn(int, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAutoWhiteBalance(true);
    std::cout << "自动白平衡: 开启" << std::endl;
}

// 自动白平衡关闭按钮回调
void CameraUI::onAutoWBOff(int, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    ui->camera.setAutoWhiteBalance(false);
    std::cout << "自动白平衡: 关闭" << std::endl;
}

// 保存配置按钮回调 (修改为按钮回调格式)
void CameraUI::onSaveConfigButtonClick(int, void* userdata) {
    CameraUI* ui = static_cast<CameraUI*>(userdata);
    // 保存配置
    ui->camera.saveParametersToConfig();
    std::cout << "相机参数已保存到配置文件" << std::endl;
}

// Empty definitions for new armor detection threshold callbacks
void CameraUI::onArmorBrightnessThresholdRedChange(int value, void* userdata) {
    // TODO: Implement or link to actual parameter update
    // reinterpret_cast<CameraUI*>(userdata)->params.armorBrightnessThresholdRed = value;
    // std::cout << "Armor Brightness Threshold Red: " << value << std::endl;
}

void CameraUI::onArmorBrightnessThresholdBlueChange(int value, void* userdata) {
    // TODO: Implement or link to actual parameter update
    // reinterpret_cast<CameraUI*>(userdata)->params.armorBrightnessThresholdBlue = value;
    // std::cout << "Armor Brightness Threshold Blue: " << value << std::endl;
}

void CameraUI::onBrightnessThresholdChange(int value, void* userdata) {
    // TODO: Implement or link to actual parameter update
    // reinterpret_cast<CameraUI*>(userdata)->params.brightnessThreshold = value;
    // std::cout << "Brightness Threshold: " << value << std::endl;
}

void CameraUI::onChannalSumThresholdChange(int value, void* userdata) {
    // TODO: Implement or link to actual parameter update
    // reinterpret_cast<CameraUI*>(userdata)->params.channalSumThreshold = value;
    // std::cout << "Channal Sum Threshold: " << value << std::endl;
}
