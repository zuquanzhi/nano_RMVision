#include "CameraUI.h"
#include <iostream>
#include <chrono>

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
    // 创建用于显示相机图像的窗口
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);
    cv::resizeWindow(windowName, 800, 600);
    
    // 创建控制相机参数的窗口
    cv::namedWindow("Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Controls", 600, 400);
    
    // 创建所有滑动条
    createTrackbars();
    
    // 创建按钮控件
    createButtons();
    
    // 从相机读取参数更新界面
    updateTrackbarsFromCamera();
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
    
    while (running) {
        // 获取相机图像
        if (camera.getFrame(frame)) {
            // 显示图像
            cv::imshow(windowName, frame);
        } else {
            std::cerr << "获取图像失败" << std::endl;
        }
        
        // 处理键盘输入
        int key = cv::waitKey(10);
        if (key == 27) { // ESC键
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