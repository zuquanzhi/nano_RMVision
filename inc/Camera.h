#ifndef SIMPLE_CAMERA_H
#define SIMPLE_CAMERA_H

#include <opencv2/opencv.hpp>
#include <string>
#include <mutex>
#include <CameraApi.h>
#include "ConfigManager.h"

// 定义相机断开连接状态常量（在SDK中未直接定义）
#define CAMERA_CONNECTION_STATUS_DISCONNECTED 0

class SimpleCamera {
private:
    int hCamera = -1;
    tSdkCameraDevInfo tCameraEnumList[1];
    tSdkCameraCapbility tCapability;
    int frame_width = 1280;
    int frame_height = 1024;
    double exposure_time = 10000.0;
    double analog_gain = 1.0;
    bool is_running = false;
    int brightness = 0;      // 亮度目标值 (0 ~ 1000)
    int gamma = 100;         // Gamma值 (0 ~ 350)
    int contrast = 6;        // 对比度 (-50 ~ 100)
    int saturation = 80;     // 饱和度 (0 ~ 200)
    int sharpness = 20;      // 锐度 (0 ~ 100)
    int r_gain = 80;         // 红色增益 (0 ~ 400)
    int g_gain = 76;         // 绿色增益 (0 ~ 400)
    int b_gain = 110;        // 蓝色增益 (0 ~ 400)
    bool auto_exposure = true;  // 自动曝光
    bool auto_wb = false;       // 自动白平衡
    std::mutex camera_mutex;
    bool reconnect_enabled = true; // 自动重连
    
    // 配置管理器
    ConfigManager config_manager;
    
    // 连接状态回调函数
    static void CameraConnectionStatusCallback(CameraHandle hCamera, UINT event, UINT uParam, PVOID context);

public:
    SimpleCamera(const std::string& configPath = "config/camera_config_info.yaml");
    ~SimpleCamera();

    bool init();
    void setupCameraParameters();
    bool getFrame(cv::Mat& frame);
    void close();

    // 参数设置函数
    void setExposureTime(double time);
    void setAnalogGain(double gain);
    void setBrightness(int value);
    void setGamma(int value);
    void setContrast(int value);
    void setSaturation(int value);
    void setSharpness(int value);
    void setRGain(int value);
    void setGGain(int value);
    void setBGain(int value);
    void setAutoExposure(bool enable);
    void setAutoWhiteBalance(bool enable);
    
    // 新增：从配置文件加载参数
    bool loadParametersFromConfig();
    
    // 新增：保存参数到配置文件
    bool saveParametersToConfig();
    
    // 获取参数值 - 供UI使用
    double getExposureTime() const { return exposure_time; }
    double getAnalogGain() const { return analog_gain; }
    int getBrightness() const { return brightness; }
    int getGamma() const { return gamma; }
    int getContrast() const { return contrast; }
    int getSaturation() const { return saturation; }
    int getSharpness() const { return sharpness; }
    int getRGain() const { return r_gain; }
    int getGGain() const { return g_gain; }
    int getBGain() const { return b_gain; }
    bool getAutoExposure() const { return auto_exposure; }
    bool getAutoWhiteBalance() const { return auto_wb; }
};

#endif // SIMPLE_CAMERA_H