#ifndef CAMERA_K
#define CAMERA_K

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "CameraApi.h"
#include "sensor_msgs/msg/image.hpp"
#include <exception>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include "interfaces/msg/readpack.hpp"
#include "../../../config/base.h"

// 初始化SDK
#define INIT_MV_SDK(success)                          \
    do                                                \
    {                                                 \
        if (!success)                                 \
        {                                             \
            throw CameraException("init sdk failed"); \
        }                                             \
    } while (0)

// 列举相机
#define ENUM_MV(success, num)                                                  \
    do                                                                         \
    {                                                                          \
        if (!success)                                                          \
        {                                                                      \
            throw CameraException("enum mv camera failed.");                   \
        }                                                                      \
        else                                                                   \
        {                                                                      \
            RCLCPP_INFO(this->get_logger(), "%dcamera device detected!", num); \
        }                                                                      \
    } while (0)

// 相机初始化
#define INIT_MV_OBJ(success)                              \
    do                                                    \
    {                                                     \
        if (!success)                                     \
        {                                                 \
            throw CameraException("Init camera failed."); \
        }                                                 \
    } while (0)

// 获得相机参数
#define GET_MV_PARAM(success, info)                      \
    do                                                   \
    {                                                    \
        if (!success)                                    \
        {                                                \
            RCLCPP_INFO(this->get_logger(), "%s", info); \
            return false;                                \
        }                                                \
    } while (0)

// 设置相机参数
#define CONFIG_MV(success, info)         \
    do                                   \
    {                                    \
        if (!success)                    \
        {                                \
            throw CameraException(info); \
        }                                \
    } while (0)

// 运行相机
#define PLAY_MV(success)                                  \
    do                                                    \
    {                                                     \
        if (!success)                                     \
        {                                                 \
            throw CameraException("camera play failed."); \
        }                                                 \
    } while (0)

// 关闭相机
#define CLOSE_MV(success, info)                                     \
    do                                                              \
    {                                                               \
        if (!success)                                               \
        {                                                           \
            RCLCPP_INFO(this->get_logger(), "close camera failed"); \
            exit(-1);                                               \
        }                                                           \
    } while (0)
enum CAMMODE
{
    CAMMODE_ARMOR = 1,

    /// 小能量机关
    CAMMODE_SMALLRUNE = 2,

    /// 大能量机关
    CAMMODE_BIGRUNE = 3,

    /// 跟随击打前哨战
    CAMMODE_DYNAMIC_OUTPOST = 4,

    /// 固定击打前哨战
    CAMMODE_FIX_OUTPOST = 5,

    CAMMODE_OUTPOST_F = 6,
};
enum CAMEnemyColor
{
    /// 默认由电控控制
    COLOR_AUTO = 0,

    /// 调试时固定红色
    COLOR_RED = 1,

    /// 调式时固定蓝色
    COLOR_BLUE = 2,

    /// 调试时两种颜色都可以识别
    COLOR_ALL = 3,
};
class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher(ImagePublisher &T) = delete;
    ImagePublisher(const ImagePublisher &T) = delete;
    ImagePublisher &operator=(ImagePublisher &T) = delete;
    ImagePublisher &operator=(const ImagePublisher &T) = delete;

    CameraHandle hCamera = -1;
    cv::Mat image_object;
    tSdkCameraCapbility tCapability;
    std::fstream camera_info_file;
    int mode = CAMMODE_ARMOR;
    int color = COLOR_AUTO;

    ImagePublisher(std::string name);

    ~ImagePublisher();

    void init();

    bool open();

    bool isOpen();

    bool close();

    void get_img(cv::Mat &img);
    void setupCameraForAimbotMode();
    void setupCameraForAutoExpMode();
    void setupCameraForHitOutpostMode();
    void setupCameraForOutpostF();
    void setupCameraForRuneMode();
    void write_camera_info();
    void RuneAdjustCameraParam();
    void setupCameraForRuneAdjustCameraParamMode(int);
    bool isStart = false;

    int car_id = 6;

    /// 帧宽
    int frame_width = 640;

    /// 帧高
    int frame_height = 480;

    /// 曝光时间, 主要是用来输出
    double exposure_time;

    /// B通道增益，默认100
    int blue_channel_gain = 100;

    /// G通道增益，默认100
    int green_channel_gain = 130;

    /// R通道增益，默认100
    int red_channel_gain = 160;

    /// 饱和度，默认100
    int saturation = 100;

    /// 对比度，默认100
    int contrast = 100;

    /// 伽马值，默认100
    int gamma = 100;

    /// 设置图像锐化程度[0-100],默认0
    int sharpness = 0;

    /*******设置手动曝光相关参数************************/
    /// 自瞄曝光时间，需要动态调节,注意是double这个数据类型
    double armor_exposure_time = 3000;

    double outpost_exposure_time = 10000;

    /// 能量机关曝光时间
    double rune_red_exposure_time = 1501;

    double rune_blue_exposure_time = 801;

    /// 模拟增益
    int analog_gain = 64;
    int outpost_gain = 64;

     /// 模拟增益
    int F_exp = 64;
    int F_gain = 64;
    /*****************END************************************/

    /****** 设置自动曝光相关参数**********************/
    /// 自动曝光目标亮度值, int
    int auto_exp_target_brightness = 55;

    /// 自动曝光阈值, int
    int auto_exp_thresh = 5;

    /// 自动曝光 曝光时间最小值, double
    double auto_exp_min_expourse_time = 100;

    /// 自动曝光 曝光时间最大值, double
    double auto_exp_max_expourse_time = 8000;

    /// 模拟增益默认设置为最高
    /// 自动曝光的模拟增益最小值, int
    int auto_exp_min_analog_gain = 128;

    /// 自动曝光的模拟增益最大值, int
    int auto_exp_max_analog_gain = 128;

    /// 大风车调试相关

    /// 能量机关相机曝光调节下限
    int RUNE_CAMERAPARAM_LOW;

    /// 能量机关相机曝光调节上限
    int RUNE_CAMERAPARAM_HIGH;

    /// 能量机关相机曝光更改时间间隔(单位：s)
    int RUNE_CAMERAPARAM_TIMEINTERVAL;

    /// 是否需要能量机关相机曝光调试
    int RUNE_CAMERAPARAM_ADJUST = 0;

    /// 能量机关相机曝光更改值间隔
    int RUNE_EXPORSURETIME_INTERVAL;

    /// 能量机关相机曝光调试基础时间
    int camera_param_adjust_base_time = 0;

    /// 能量机关相机曝光时间调整目标值
    int camera_base_exposuretime = 0;

    /// 能量机关相机曝光时间显示值（当前视频曝光真实值）
    int current_true_camera_exposuretime;

    int use_video = 0;
    std::string video_path;
    cv::VideoCapture capture;

private:
    void timer_callback();

    rclcpp::TimerBase::SharedPtr timer_;
    void readpack_callback(const interfaces::msg::Readpack::SharedPtr msg);
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_;
    rclcpp::Subscription<interfaces::msg::Readpack>::SharedPtr readpack_subscribe_;
};

class CameraException : public std::exception
{
private:
    /// 异常信息字符串
    std::string e_what;

public:
    CameraException() = default;

    /**
     * @brief 自定义构造函数, 需要给出异常信息
     *
     * @param error 异常描述信息
     */
    CameraException(const std::string &error) : e_what(error) {}

    ~CameraException() throw() {}

    /**
     * @brief 异常规格说明：不抛出异常
     *
     * @return 异常信息字符串
     * @note 该函数为std::exception类中的覆盖
     */
    virtual const char *what() const throw() { return e_what.c_str(); }
};

#endif