#include <camdrv/camdrv.hpp>

ImagePublisher::~ImagePublisher() { close(); }

bool ImagePublisher::close()
{
    if (hCamera < 0)
    {

        RCLCPP_INFO(this->get_logger(), "camera already closed");

        return true;
    }
    if (!(CameraUnInit(hCamera) == CAMERA_STATUS_SUCCESS))
    {
        return false;
    }
    hCamera = -1;
    RCLCPP_INFO(this->get_logger(), "close camera success.");
    return true;
}
bool ImagePublisher::isOpen() { return (hCamera > 0); }
void ImagePublisher::init()
{
    this->declare_parameter("g_mv_camera_config_path", "src/config/mv_camera_config6.xml");
    std::string mv_camera_config_path = this->get_parameter("g_mv_camera_config_path").as_string();

    this->declare_parameter("g_other_param_path", "src/config/other.xml");
    std::string g_other_param_path = this->get_parameter("g_other_param_path").as_string();

    cv::FileStorage config(g_other_param_path, cv::FileStorage::READ);
    cv::FileNode config_node = config["workspace"];

    use_video = config_node["USE_VIDEO"];
    std::string vd, vn;
    config_node["VIDEO_DIR"] >> vd;
    config_node["VIDEO_NAME"] >> vn;

    if (use_video == 1)
        video_path = vd + vn;

    cv::FileStorage file(mv_camera_config_path, cv::FileStorage::READ);
    if (!file.isOpened())
    {
        RCLCPP_INFO(this->get_logger(), "can't open camera config file.");
        abort();
    }

    file["frame_width"] >> frame_width;
    file["frame_height"] >> frame_height;
    file["blue_channel_gain"] >> blue_channel_gain;
    file["green_channel_gain"] >> green_channel_gain;
    file["red_channel_gain"] >> red_channel_gain;
    file["saturation"] >> saturation;
    file["contrast"] >> contrast;
    file["gamma"] >> gamma;
    file["sharpness"] >> sharpness;

    file["auto_exp_target_brightness"] >> auto_exp_target_brightness;
    file["auto_exp_thresh"] >> auto_exp_thresh;
    file["auto_exp_min_expourse_time"] >> auto_exp_min_expourse_time;
    file["auto_exp_max_expourse_time"] >> auto_exp_max_expourse_time;
    RCLCPP_INFO(this->get_logger(), "auto_exp_max_expourse_time: %lf", auto_exp_max_expourse_time);
    file["auto_exp_min_analog_gain"] >> auto_exp_min_analog_gain;
    file["auto_exp_max_analog_gain"] >> auto_exp_max_analog_gain;

    file["armor_exposure_time"] >> armor_exposure_time;
    file["outpost_exposure_time"] >> outpost_exposure_time;
    file["rune_red_exposure_time"] >> rune_red_exposure_time;
    file["rune_blue_exposure_time"] >> rune_blue_exposure_time;
    file["analog_gain"] >> analog_gain;
    file["outpost_analog_gain"] >> outpost_gain;

    file["F_exposure_time"] >> F_exp;
    file["F_analog_gain"] >> F_gain;
    file["RUNE_CAMERAPARAM_LOW"] >> RUNE_CAMERAPARAM_LOW;
    file["RUNE_CAMERAPARAM_HIGH"] >> RUNE_CAMERAPARAM_HIGH;
    file["RUNE_CAMERAPARAM_TIMEINTERVAL"] >> RUNE_CAMERAPARAM_TIMEINTERVAL;
    file["RUNE_EXPORSURETIME_INTERVAL"] >> RUNE_EXPORSURETIME_INTERVAL;
    file["RUNE_CAMERAPARAM_ADJUST"] >> RUNE_CAMERAPARAM_ADJUST;
}
bool ImagePublisher::open()
{
    if (isOpen())
    {
        RCLCPP_INFO(this->get_logger(), "camera is already opened.");
        if (!close())
        {
            RCLCPP_INFO(this->get_logger(), "close camera failed.");
            return false;
        }
    }

    int cameraCounts = 1;
    /// 工业摄像头设备列表数组
    tSdkCameraDevInfo tCameraEnumList[1];
    // 初始化SDK
    INIT_MV_SDK((CameraSdkInit(1) == CAMERA_STATUS_SUCCESS)); // 修改为1，表示只打开一台相机，减少枚举时间
    // 枚举设备，并建立设备列表
    ENUM_MV((CameraEnumerateDevice(tCameraEnumList, &cameraCounts) == CAMERA_STATUS_SUCCESS), cameraCounts);
    
    if (cameraCounts <= 0) {
        RCLCPP_ERROR(this->get_logger(), "无法找到相机设备，请检查连接！");
        return false;
    }
    
    // 相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    INIT_MV_OBJ((CameraInit(&tCameraEnumList[0], -1, -1, &hCamera) == CAMERA_STATUS_SUCCESS));
    // 获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    GET_MV_PARAM((CameraGetCapability(hCamera, &tCapability) == CAMERA_STATUS_SUCCESS), "get camera capability failed");
    
    // 设置掉线自动重连
    CONFIG_MV((CameraSetConnectionStatusCallback(hCamera, nullptr, (PVOID)this) == CAMERA_STATUS_SUCCESS), 
              "set connect-status-callback func");
    
    // 设定相机触发模式为软触发
    CONFIG_MV((CameraSetTriggerMode(hCamera, 1) == CAMERA_STATUS_SUCCESS), "set trigger mode");

    // 配置分辨率
    tSdkImageResolution imageResolution; // 相机的分辨率描述
    GET_MV_PARAM((CameraGetImageResolution(hCamera, &imageResolution) == CAMERA_STATUS_SUCCESS), "get camera resolution failed");
    imageResolution.iIndex = 0XFF;
    imageResolution.iWidth = frame_width;
    imageResolution.iHeight = frame_height;
    // 根据参数文件中的分辨率来判断是否需要取roi
    int xdelta = -1;
    int ydelta = -1;
    xdelta = (imageResolution.iWidthFOV - frame_width) / 2;
    ydelta = (imageResolution.iHeightFOV - frame_height) / 2;
    if (xdelta > 0 && xdelta < imageResolution.iWidthFOV)
        imageResolution.iHOffsetFOV = xdelta;
    if (ydelta > 0 && ydelta < imageResolution.iHeightFOV)
        imageResolution.iVOffsetFOV = ydelta;
    // 设置图像的分辨率
    CONFIG_MV((CameraSetImageResolution(hCamera, &imageResolution) == CAMERA_STATUS_SUCCESS), "set camera resolution failed");
    // 设置帧率
    CONFIG_MV((CameraSetFrameSpeed(hCamera, tCapability.iFrameSpeedDesc - 1) == CAMERA_STATUS_SUCCESS), "set camera fps mode failed");
    // 设置gamma
    CONFIG_MV((CameraSetGamma(hCamera, gamma) == CAMERA_STATUS_SUCCESS), "set camera gamma failed");
    // 设置对比度CameraGetImageBuffer
    CONFIG_MV((CameraSetNoiseFilter(hCamera, true) == CAMERA_STATUS_SUCCESS), "set camera noise-filter failed");
    CONFIG_MV((CameraSetGain(hCamera, red_channel_gain, green_channel_gain, blue_channel_gain) == CAMERA_STATUS_SUCCESS), "set camera color channel gain failed");
    // 设置图像锐化
    CONFIG_MV((CameraSetSharpness(hCamera, sharpness) == CAMERA_STATUS_SUCCESS), "set camera sharpness failed");
    // 用自瞄的相机曝光和模拟增益设置进行初始化
    setupCameraForAimbotMode();
    // 8位三通道
    CONFIG_MV((CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8) == CAMERA_STATUS_SUCCESS), "set camera output format failed");
    // 输出相机工作信息，确定相机处于正常工作状态表
    // write_camera_info();
    PLAY_MV((CameraPlay(hCamera) == CAMERA_STATUS_SUCCESS));

    if (use_video)
        capture.open(video_path);

    RCLCPP_INFO(this->get_logger(), "open camera success.");
    return true;
}
void ImagePublisher::setupCameraForAimbotMode()
{
    setupCameraForAutoExpMode();
}
void ImagePublisher::setupCameraForAutoExpMode()
{
    CONFIG_MV((CameraSetAeState(hCamera, true) == CAMERA_STATUS_SUCCESS), "set camera auto exp mode failed");
    CONFIG_MV((CameraSetAeTarget(hCamera, auto_exp_target_brightness) == CAMERA_STATUS_SUCCESS), "set camera target brightness failed");
    CONFIG_MV((CameraSetAeExposureRange(hCamera, auto_exp_min_expourse_time, auto_exp_max_expourse_time) == CAMERA_STATUS_SUCCESS), "set camera exp range failed");
    CONFIG_MV((CameraSetAeAnalogGainRange(hCamera, auto_exp_min_analog_gain, auto_exp_max_analog_gain) == CAMERA_STATUS_SUCCESS), "set camera analog gain range failed");
    CONFIG_MV((CameraSetAeThreshold(hCamera, auto_exp_thresh) == CAMERA_STATUS_SUCCESS), "set camera auto exp thresh failed");
}

void ImagePublisher::setupCameraForHitOutpostMode()
{
    // CONFIG_MV((CameraSetAeState(hCamera, true) == CAMERA_STATUS_SUCCESS), "set camera auto exp mode failed");
    // CONFIG_MV((CameraSetAeTarget(hCamera, auto_exp_target_brightness) == CAMERA_STATUS_SUCCESS), "set camera target brightness failed");
    // CONFIG_MV((CameraSetAeExposureRange(hCamera, auto_exp_min_expourse_time, auto_exp_max_expourse_time) == CAMERA_STATUS_SUCCESS), "set camera exp range failed");
    // CONFIG_MV((CameraSetAeAnalogGainRange(hCamera, auto_exp_min_analog_gain, auto_exp_max_analog_gain) == CAMERA_STATUS_SUCCESS), "set camera analog gain range failed");
    // CONFIG_MV((CameraSetAeThreshold(hCamera, auto_exp_thresh) == CAMERA_STATUS_SUCCESS), "set camera auto exp thresh failed");
    CONFIG_MV((CameraSetAeState(hCamera, false) == CAMERA_STATUS_SUCCESS), "set camera fixed exp mode failed");
    CONFIG_MV((CameraSetExposureTime(hCamera, outpost_exposure_time) == CAMERA_STATUS_SUCCESS), "set camera exp time failed");
    CONFIG_MV((CameraSetAnalogGain(hCamera, outpost_gain) == CAMERA_STATUS_SUCCESS), "set camera analog gain failed");
}
void ImagePublisher::setupCameraForRuneMode()
{
    CONFIG_MV((CameraSetAeState(hCamera, false) == CAMERA_STATUS_SUCCESS), "set camera fixed exp mode failed");
    if (color == CAMEnemyColor::COLOR_BLUE)
    {
        CONFIG_MV((CameraSetExposureTime(hCamera, rune_red_exposure_time) == CAMERA_STATUS_SUCCESS), "set camera exp time failed");
    }
    else
    {
        CONFIG_MV((CameraSetExposureTime(hCamera, rune_blue_exposure_time) == CAMERA_STATUS_SUCCESS), "set camera exp time failed");
    }
    CONFIG_MV((CameraSetAnalogGain(hCamera, analog_gain) == CAMERA_STATUS_SUCCESS), "set camera analog gain failed");
}

void ImagePublisher::setupCameraForOutpostF()
{
    CONFIG_MV((CameraSetAeState(hCamera, false) == CAMERA_STATUS_SUCCESS), "set camera fixed exp mode failed");
    CONFIG_MV((CameraSetExposureTime(hCamera, F_exp) == CAMERA_STATUS_SUCCESS), "set camera exp time failed");
    CONFIG_MV((CameraSetAnalogGain(hCamera, F_gain) == CAMERA_STATUS_SUCCESS), "set camera analog gain failed");
}

void ImagePublisher::write_camera_info()
{
    camera_info_file.open("/home/kuang/project/herocv-2024-ver-ros/src/camdrv/config/camera_config_info.yaml", std::ios::out | std::ios::trunc);
    if (!camera_info_file.is_open())
    {
        RCLCPP_INFO(this->get_logger(), "camera file open error!");
    }
    // 相机状态
    camera_info_file << "camera_status: " << (hCamera > 0) << std::endl
                     << std::endl;
    // 分辨率信息
    tSdkImageResolution imageResolution;
    CameraGetImageResolution(hCamera, &imageResolution);
    camera_info_file << "resolution: (" << imageResolution.iWidth << ", " << imageResolution.iHeight << ")" << std::endl;

    // 曝光模式
    int auto_exp = 0;
    CameraGetAeState(hCamera, &auto_exp);
    camera_info_file << "auto exp status: " << auto_exp << std::endl;
    // 自动曝光目标亮度值
    CameraGetAeTarget(hCamera, &auto_exp_target_brightness);
    camera_info_file << "auto exp aim brightness: " << auto_exp_target_brightness << std::endl;
    // 自动曝光阈值
    CameraGetAeThreshold(hCamera, &auto_exp_thresh);
    camera_info_file << "auto exp thresh: " << auto_exp_thresh << std::endl;
    // 自动曝光曝光范围
    CameraGetAeExposureRange(hCamera, &auto_exp_min_expourse_time, &auto_exp_max_expourse_time);
    camera_info_file << "auto exp time range: "
                     << "(" << auto_exp_min_expourse_time << "," << auto_exp_max_expourse_time << ")" << std::endl;
    // 自动曝光相机模拟增益
    CameraGetAeAnalogGainRange(hCamera, &auto_exp_min_analog_gain, &auto_exp_max_analog_gain);
    camera_info_file << "auto exp analog gain: "
                     << "(" << auto_exp_min_analog_gain << "," << auto_exp_max_analog_gain << ")" << std::endl;

    // gamma值信息
    CameraGetGamma(hCamera, &gamma);
    camera_info_file << "gamma: " << gamma << std::endl;
    // 对比度信息
    CameraGetContrast(hCamera, &contrast);
    camera_info_file << "contrast: " << contrast << std::endl;
    // 模拟增益值信息
    CameraGetAnalogGain(hCamera, &analog_gain);
    camera_info_file << "fix exp analog gain: " << analog_gain << std::endl;
    // 图像锐化信息
    CameraGetSharpness(hCamera, &sharpness);
    camera_info_file << "sharpness: " << sharpness << std::endl;
    camera_info_file << "number of frame speed models: " << tCapability.iFrameSpeedDesc << std::endl;
    camera_info_file.close();
}

void ImagePublisher::get_img(cv::Mat &img)
{
    if (!isOpen())
    {
        throw CameraException("Get image failed. Camera is not opened.");
    }
    tSdkFrameHead head;
    BYTE *buffer = nullptr;
    //触发一次
    CameraSoftTrigger(hCamera);
    CONFIG_MV((CameraGetImageBuffer(hCamera, &head, &buffer, 10000) == CAMERA_STATUS_SUCCESS), "get img from buffer failed");
    img = cv::Mat(cv::Size(head.iWidth, head.iHeight), CV_8UC3);
    CONFIG_MV((CameraImageProcess(hCamera, buffer, img.data, &head) == CAMERA_STATUS_SUCCESS), "camera img process failed");
    CONFIG_MV((CameraReleaseImageBuffer(hCamera, buffer) == CAMERA_STATUS_SUCCESS), "release img buffer failed");
}

void ImagePublisher::RuneAdjustCameraParam()
{
    if (RUNE_CAMERAPARAM_ADJUST)
    {
        // cv::Mat t;
        // this->get_img(t);
        // cv::putText(t, std::to_string(current_true_camera_exposuretime), cv::Point2f(10, 400), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 0));
        if (camera_param_adjust_base_time == 0)
        {
            camera_param_adjust_base_time = rclcpp::Clock().now().seconds();
        }
        if (camera_base_exposuretime == 0)
        {
            camera_base_exposuretime = RUNE_CAMERAPARAM_LOW;
        }
        if ((rclcpp::Clock().now().seconds() - camera_param_adjust_base_time) > RUNE_CAMERAPARAM_TIMEINTERVAL)
        {
            this->setupCameraForRuneAdjustCameraParamMode(camera_base_exposuretime);
            current_true_camera_exposuretime = camera_base_exposuretime;
            camera_base_exposuretime += RUNE_EXPORSURETIME_INTERVAL;
            if (camera_base_exposuretime > RUNE_CAMERAPARAM_HIGH)
            {
                camera_base_exposuretime = RUNE_CAMERAPARAM_LOW;
            }
            camera_param_adjust_base_time = rclcpp::Clock().now().seconds();
            RCLCPP_INFO(rclcpp::get_logger("node_camdrv"), "rune_exporsurt:%d", camera_base_exposuretime);
        }
        // cv::imshow("test", t);
        // cv::waitKey(10);
    }
    else
    {
        return;
    }
}
void ImagePublisher::setupCameraForRuneAdjustCameraParamMode(int exposure_time)
{
    CONFIG_MV((CameraSetAeState(hCamera, false) == CAMERA_STATUS_SUCCESS), "set rune_camera fixed exp mode failed");
    CONFIG_MV((CameraSetExposureTime(hCamera, exposure_time) == CAMERA_STATUS_SUCCESS), "set rune_camera exp time failed");
    CONFIG_MV((CameraSetAnalogGain(hCamera, analog_gain) == CAMERA_STATUS_SUCCESS), "set rune_camera analog gain failed");
}
