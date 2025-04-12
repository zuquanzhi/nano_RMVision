#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <chrono>
#include <fstream>
#include <CameraApi.h>

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
    
    // 连接状态回调函数，修正参数列表与SDK定义匹配
    static void CameraConnectionStatusCallback(CameraHandle hCamera, UINT event, UINT uParam, PVOID context) {
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

public:
    SimpleCamera() = default;
    ~SimpleCamera() { close(); }

    bool init() {
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

    void setupCameraParameters() {
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

    bool getFrame(cv::Mat& frame) {
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

    void setExposureTime(double time) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        exposure_time = time;
        if (!auto_exposure) {
            CameraSetExposureTime(hCamera, exposure_time);
        }
    }

    void setAnalogGain(double gain) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        analog_gain = gain;
        CameraSetAnalogGain(hCamera, analog_gain);
    }

    // 新增的参数设置函数
    void setBrightness(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        brightness = value;
        // 修正：使用CameraSetAeTarget设置目标亮度值，而不是Gamma
        CameraSetAeTarget(hCamera, brightness);
    }
    
    void setGamma(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        // 增加Gamma参数
        CameraSetGamma(hCamera, value);
    }

    void setContrast(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        contrast = value;
        CameraSetContrast(hCamera, contrast);
    }

    void setSaturation(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        saturation = value;
        CameraSetSaturation(hCamera, saturation);
    }

    void setSharpness(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        sharpness = value;
        CameraSetSharpness(hCamera, sharpness);
    }

    void setRGain(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        r_gain = value;
        if (!auto_wb) {
            CameraSetGain(hCamera, r_gain, g_gain, b_gain);
        }
    }

    void setGGain(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        g_gain = value;
        if (!auto_wb) {
            CameraSetGain(hCamera, r_gain, g_gain, b_gain);
        }
    }

    void setBGain(int value) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        b_gain = value;
        if (!auto_wb) {
            CameraSetGain(hCamera, r_gain, g_gain, b_gain);
        }
    }

    void setAutoExposure(bool enable) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        auto_exposure = enable;
        CameraSetAeState(hCamera, enable ? TRUE : FALSE);
    }

    void setAutoWhiteBalance(bool enable) {
        std::lock_guard<std::mutex> lock(camera_mutex);
        auto_wb = enable;
        CameraSetWbMode(hCamera, enable ? TRUE : FALSE);
    }

    void close() {
        if (hCamera >= 0) {
            CameraUnInit(hCamera);
            hCamera = -1;
        }
    }
};

int main() {
    SimpleCamera camera;
    if (!camera.init()) {
        std::cerr << "相机初始化失败" << std::endl;
        return -1;
    }

    // 创建主窗口和控制面板窗口
    cv::namedWindow("Camera", cv::WINDOW_NORMAL);
    cv::namedWindow("Controls", cv::WINDOW_NORMAL);
    cv::resizeWindow("Controls", 400, 700);  // 设置控制面板窗口大小
    cv::resizeWindow("Camera", 640, 480);  // 设置相机窗口的初始大小

    // 曝光和增益控制 - 增大曝光范围到10000ms(10秒)
    int exp_trackbar = 15;  // 初始曝光值15ms
    cv::createTrackbar("曝光时间(ms)", "Controls", &exp_trackbar, 10000, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setExposureTime(pos * 1000.0);  // 转换为微秒
    }, &camera);
    // 立即应用初始曝光设置
    camera.setExposureTime(exp_trackbar * 1000.0);

    int gain_trackbar = 15;  // 初始增益值1.5
    cv::createTrackbar("增益", "Controls", &gain_trackbar, 100, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setAnalogGain(pos / 10.0);
    }, &camera);
    // 立即应用初始增益设置
    camera.setAnalogGain(gain_trackbar / 10.0);

    // 亮度目标值控制 (0 ~ 1000)
    int brightness_trackbar = 50;  // 默认值50
    cv::createTrackbar("亮度目标值", "Controls", &brightness_trackbar, 1000, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setBrightness(pos);  // 设置亮度目标值
    }, &camera);
    // 立即应用亮度目标值设置
    camera.setBrightness(brightness_trackbar);
    
    // 添加Gamma值控制 (0 ~ 350)
    int gamma_trackbar = 100;  // 默认值100，约等于Gamma 1.0
    cv::createTrackbar("Gamma值", "Controls", &gamma_trackbar, 350, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setGamma(pos);  // 设置Gamma值
    }, &camera);
    // 立即应用Gamma设置
    camera.setGamma(gamma_trackbar);

    // 对比度控制 (-50 ~ 100)
    int contrast_trackbar = 75;  // 映射到[-50,100]范围的中间值
    cv::createTrackbar("对比度", "Controls", &contrast_trackbar, 150, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setContrast(pos - 50);  // 转换到 -50~100 范围
    }, &camera);
    // 立即应用对比度设置
    camera.setContrast(contrast_trackbar - 50);

    // 饱和度控制 (0 ~ 200)
    int saturation_trackbar = 100;  // 默认值100
    cv::createTrackbar("饱和度", "Controls", &saturation_trackbar, 200, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setSaturation(pos);
    }, &camera);
    // 立即应用饱和度设置
    camera.setSaturation(saturation_trackbar);

    // 锐度控制 (0 ~ 100)
    int sharpness_trackbar = 20;  // 默认20，适中的锐度
    cv::createTrackbar("锐度", "Controls", &sharpness_trackbar, 100, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setSharpness(pos);
    }, &camera);
    // 立即应用锐度设置
    camera.setSharpness(sharpness_trackbar);

    // 白平衡控制 (0 ~ 400)
    int r_gain_trackbar = 80;   // 红色增益
    int g_gain_trackbar = 76;   // 绿色增益
    int b_gain_trackbar = 110;  // 蓝色增益
    cv::createTrackbar("红色增益", "Controls", &r_gain_trackbar, 400, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setRGain(pos);
    }, &camera);
    cv::createTrackbar("绿色增益", "Controls", &g_gain_trackbar, 400, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setGGain(pos);
    }, &camera);
    cv::createTrackbar("蓝色增益", "Controls", &b_gain_trackbar, 400, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setBGain(pos);
    }, &camera);
    // 立即应用白平衡设置
    camera.setRGain(r_gain_trackbar);
    camera.setGGain(g_gain_trackbar);
    camera.setBGain(b_gain_trackbar);

    // 自动曝光开关
    int auto_exp_trackbar = 1;  // 默认开启自动曝光
    cv::createTrackbar("自动曝光", "Controls", &auto_exp_trackbar, 1, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setAutoExposure(pos == 1);
    }, &camera);
    // 立即应用自动曝光设置
    camera.setAutoExposure(auto_exp_trackbar == 1);

    // 自动白平衡开关
    int auto_wb_trackbar = 0;  // 默认关闭自动白平衡
    cv::createTrackbar("自动白平衡", "Controls", &auto_wb_trackbar, 1, [](int pos, void* userdata) {
        SimpleCamera* cam = static_cast<SimpleCamera*>(userdata);
        cam->setAutoWhiteBalance(pos == 1);
    }, &camera);
    // 立即应用自动白平衡设置
    camera.setAutoWhiteBalance(auto_wb_trackbar == 1);

    cv::Mat frame;
    while (true) {
        if (camera.getFrame(frame)) {
            if (!frame.empty()) {
                // 确保帧不是空的
                
                // 在窗口上显示一些状态信息
                std::string status = "相机状态: ";
                status += auto_exp_trackbar ? "自动曝光开启" : "手动曝光";
                status += " | 白平衡: ";
                status += auto_wb_trackbar ? "自动" : "手动";
                cv::putText(frame, status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 
                            0.7, cv::Scalar(0, 255, 0), 2);
                
                cv::imshow("Camera", frame);
            } else {
                std::cerr << "获取到空帧" << std::endl;
            }
        } else {
            std::cerr << "获取帧失败" << std::endl;
            // 短暂休眠，避免在出错时CPU使用率过高
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        char key = cv::waitKey(10);  // 延长等待时间以减少CPU使用率
        if (key == 27) { // ESC键退出
            break;
        } else if (key == 's' || key == 'S') {
            // 添加保存图像功能
            if (!frame.empty()) {
                std::string filename = "camera_capture_" + 
                                     std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + 
                                     ".jpg";
                cv::imwrite(filename, frame);
                std::cout << "图像已保存为: " << filename << std::endl;
            }
        }
    }

    camera.close();
    cv::destroyAllWindows();
    return 0;
}