#include "SimpleCamera.h"
#include <iostream>
#include <thread>
#include <chrono>

// 连接状态回调函数
void SimpleCamera::CameraConnectionStatusCallback(CameraHandle hCamera, UINT event, UINT uParam, PVOID context) {
    SimpleCamera* camera = static_cast<SimpleCamera*>(context);
    if (event == CAMERA_CONNECTION_STATUS_DISCONNECTED) {
        std::cerr << "相机断开连接！" << std::endl;
        if (camera->reconnect_enabled) {
            // 尝试重新连接
            std::this_thread::sleep_for(std::chrono::seconds(1));
            std::cout << "正在尝试重新连接相机..." << std::endl;
            camera->close();
            camera->init();
        }
    }
}

SimpleCamera::SimpleCamera(const std::string& configPath) 
    : config_manager(configPath) {
}

SimpleCamera::~SimpleCamera() { 
    // 保存参数到配置文件
    saveParametersToConfig();
    close(); 
}

bool SimpleCamera::loadParametersFromConfig() {
    // 加载配置文件
    if (!config_manager.loadConfig()) {
        std::cerr << "加载配置文件失败，使用默认参数" << std::endl;
        return false;
    }
    
    // 读取分辨率
    config_manager.readInt("resolution_width", frame_width);
    config_manager.readInt("resolution_height", frame_height);
    
    // 读取曝光相关参数
    config_manager.readBool("auto_exposure", auto_exposure);
    config_manager.readDouble("exposure_time", exposure_time);
    config_manager.readDouble("analog_gain", analog_gain);
    config_manager.readInt("brightness", brightness);
    
    // 读取图像调整参数
    config_manager.readInt("gamma", gamma);
    config_manager.readInt("contrast", contrast);
    config_manager.readInt("saturation", saturation);
    config_manager.readInt("sharpness", sharpness);
    
    // 读取白平衡相关参数
    config_manager.readBool("auto_white_balance", auto_wb);
    config_manager.readInt("r_gain", r_gain);
    config_manager.readInt("g_gain", g_gain);
    config_manager.readInt("b_gain", b_gain);
    
    std::cout << "从配置文件加载参数成功" << std::endl;
    return true;
}

bool SimpleCamera::saveParametersToConfig() {
    // 准备保存配置
    if (!config_manager.saveConfig()) {
        std::cerr << "创建配置文件失败" << std::endl;
        return false;
    }
    
    // 保存分辨率
    config_manager.writeInt("camera_status", 1); // 确保有camera_status参数
    config_manager.writeInt("resolution_width", frame_width);
    config_manager.writeInt("resolution_height", frame_height);
    
    // 保存曝光相关参数
    config_manager.writeBool("auto_exposure", auto_exposure);
    config_manager.writeDouble("exposure_time", exposure_time);
    config_manager.writeDouble("analog_gain", analog_gain);
    config_manager.writeInt("brightness", brightness);
    
    // 保存图像调整参数
    config_manager.writeInt("gamma", gamma);
    config_manager.writeInt("contrast", contrast);
    config_manager.writeInt("saturation", saturation);
    config_manager.writeInt("sharpness", sharpness);
    
    // 保存白平衡相关参数
    config_manager.writeBool("auto_white_balance", auto_wb);
    config_manager.writeInt("r_gain", r_gain);
    config_manager.writeInt("g_gain", g_gain);
    config_manager.writeInt("b_gain", b_gain);
    
    // 关闭配置文件，确保数据被写入
    config_manager.closeConfigFile();
    
    std::cout << "参数已保存到配置文件" << std::endl;
    return true;
}

bool SimpleCamera::init() {
    // 从配置文件加载参数
    loadParametersFromConfig();
    
    // 初始化SDK
    int status = CameraSdkInit(1); // 修改为1，表示只枚举一台相机
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "SDK初始化失败，错误码: " << status << std::endl;
        return false;
    }

    // 枚举设备
    int cameraCounts = 1;
    status = CameraEnumerateDevice(tCameraEnumList, &cameraCounts);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "枚举设备失败，错误码: " << status << std::endl;
        return false;
    }

    if (cameraCounts <= 0) {
        std::cerr << "未找到相机设备" << std::endl;
        return false;
    }

    std::cout << "找到 " << cameraCounts << " 台相机，正在初始化第一台..." << std::endl;

    // 初始化相机 - 使用索引0初始化第一台相机
    status = CameraInit(&tCameraEnumList[0], -1, -1, &hCamera);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "相机初始化失败，错误码: " << status << std::endl;
        return false;
    }

    std::cout << "相机初始化成功，句柄: " << hCamera << std::endl;

    // 获取相机能力描述
    status = CameraGetCapability(hCamera, &tCapability);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "获取相机能力描述失败，错误码: " << status << std::endl;
        CameraUnInit(hCamera);
        hCamera = -1;
        return false;
    }

    // 设置断线自动重连回调
    status = CameraSetConnectionStatusCallback(hCamera, &SimpleCamera::CameraConnectionStatusCallback, this);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "设置连接状态回调失败，错误码: " << status << std::endl;
        // 继续执行，不返回失败
    }

    // 设置相机触发模式为软触发
    status = CameraSetTriggerMode(hCamera, 1);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "设置触发模式失败，错误码: " << status << std::endl;
        CameraUnInit(hCamera);
        hCamera = -1;
        return false;
    }

    // 设置相机参数
    setupCameraParameters();

    // 开始采集
    status = CameraPlay(hCamera);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "开始采集失败，错误码: " << status << std::endl;
        CameraUnInit(hCamera);
        hCamera = -1;
        return false;
    }

    std::cout << "相机启动成功，开始采集图像" << std::endl;
    return true;
}

void SimpleCamera::setupCameraParameters() {
    // 设置分辨率
    tSdkImageResolution imageResolution;
    CameraGetImageResolution(hCamera, &imageResolution);
    imageResolution.iIndex = 0XFF;
    imageResolution.iWidth = frame_width;
    imageResolution.iHeight = frame_height;
    
    // 自动计算ROI中心裁剪
    int xdelta = (imageResolution.iWidthFOV - frame_width) / 2;
    int ydelta = (imageResolution.iHeightFOV - frame_height) / 2;
    if (xdelta > 0 && xdelta < imageResolution.iWidthFOV)
        imageResolution.iHOffsetFOV = xdelta;
    if (ydelta > 0 && ydelta < imageResolution.iHeightFOV)
        imageResolution.iVOffsetFOV = ydelta;
        
    CameraSetImageResolution(hCamera, &imageResolution);

    // 设置输出格式为BGR8
    CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);

    // 设置帧率为最高
    CameraSetFrameSpeed(hCamera, tCapability.iFrameSpeedDesc - 1);
    
    // 设置噪声滤波
    CameraSetNoiseFilter(hCamera, true);

    // 设置曝光控制
    if (auto_exposure) {
        CameraSetAeState(hCamera, TRUE);
        // 设置目标亮度
        CameraSetAeTarget(hCamera, brightness);
        // 设置曝光范围
        CameraSetAeExposureRange(hCamera, 100, 10000);
        // 设置增益范围
        CameraSetAeAnalogGainRange(hCamera, 0, 100);
    } else {
        CameraSetAeState(hCamera, FALSE);
        // 设置曝光时间
        CameraSetExposureTime(hCamera, exposure_time);
    }

    // 设置模拟增益
    CameraSetAnalogGain(hCamera, analog_gain);

    // 设置Gamma
    CameraSetGamma(hCamera, gamma);
    
    // 设置对比度
    CameraSetContrast(hCamera, contrast);
    
    // 设置饱和度
    CameraSetSaturation(hCamera, saturation);
    
    // 设置锐度
    CameraSetSharpness(hCamera, sharpness);
    
    // 设置白平衡
    if (auto_wb) {
        CameraSetWbMode(hCamera, TRUE);
    } else {
        CameraSetWbMode(hCamera, FALSE);
        CameraSetGain(hCamera, r_gain, g_gain, b_gain);
    }
}

bool SimpleCamera::getFrame(cv::Mat& frame) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    if (hCamera < 0) {
        std::cerr << "相机未初始化" << std::endl;
        return false;
    }
    
    tSdkFrameHead frameHead;
    BYTE* pbyBuffer = NULL;
    
    // 触发一次采集
    CameraSoftTrigger(hCamera);
    
    // 设置获取图像的超时时间
    int status = CameraGetImageBuffer(hCamera, &frameHead, &pbyBuffer, 1000);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "获取图像失败，错误码: " << status << std::endl;
        return false;
    }
    
    // 验证缓冲区
    if (pbyBuffer == NULL) {
        std::cerr << "获取到的图像缓冲区为NULL" << std::endl;
        return false;
    }
    
    try {
        // 创建一个新的Mat对象直接接收BGR数据
        frame = cv::Mat(frameHead.iHeight, frameHead.iWidth, CV_8UC3);
        
        // 直接使用SDK函数进行图像处理
        status = CameraImageProcess(hCamera, pbyBuffer, frame.data, &frameHead);
        if (status != CAMERA_STATUS_SUCCESS) {
            std::cerr << "图像处理失败，错误码: " << status << std::endl;
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
            return false;
        }
        
        // 使用SDK函数释放相机缓冲区
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        
        // 检查返回的图像
        if (frame.empty()) {
            std::cerr << "转换后的帧为空" << std::endl;
            return false;
        }
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "处理图像异常: " << e.what() << std::endl;
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        return false;
    }
}

void SimpleCamera::setExposureTime(double time) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    exposure_time = time;
    if (!auto_exposure) {
        CameraSetExposureTime(hCamera, exposure_time);
    }
}

void SimpleCamera::setAnalogGain(double gain) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    analog_gain = gain;
    CameraSetAnalogGain(hCamera, analog_gain);
}

void SimpleCamera::setBrightness(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    brightness = value;
    // 修正：使用CameraSetAeTarget设置目标亮度值，而不是Gamma
    CameraSetAeTarget(hCamera, brightness);
}

void SimpleCamera::setGamma(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    gamma = value;
    CameraSetGamma(hCamera, value);
}

void SimpleCamera::setContrast(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    contrast = value;
    CameraSetContrast(hCamera, contrast);
}

void SimpleCamera::setSaturation(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    saturation = value;
    CameraSetSaturation(hCamera, saturation);
}

void SimpleCamera::setSharpness(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    sharpness = value;
    CameraSetSharpness(hCamera, sharpness);
}

void SimpleCamera::setRGain(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    r_gain = value;
    if (!auto_wb) {
        CameraSetGain(hCamera, r_gain, g_gain, b_gain);
    }
}

void SimpleCamera::setGGain(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    g_gain = value;
    if (!auto_wb) {
        CameraSetGain(hCamera, r_gain, g_gain, b_gain);
    }
}

void SimpleCamera::setBGain(int value) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    b_gain = value;
    if (!auto_wb) {
        CameraSetGain(hCamera, r_gain, g_gain, b_gain);
    }
}

void SimpleCamera::setAutoExposure(bool enable) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    auto_exposure = enable;
    CameraSetAeState(hCamera, enable ? TRUE : FALSE);
}

void SimpleCamera::setAutoWhiteBalance(bool enable) {
    std::lock_guard<std::mutex> lock(camera_mutex);
    auto_wb = enable;
    CameraSetWbMode(hCamera, enable ? TRUE : FALSE);
}

void SimpleCamera::close() {
    if (hCamera >= 0) {
        CameraUnInit(hCamera);
        hCamera = -1;
    }
}