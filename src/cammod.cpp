#include "camdrv/camdrv.hpp"

void ImagePublisher::readpack_callback(const interfaces::msg::Readpack::SharedPtr msg)
{
    if (this->mode != msg->mode)
    {
        this->mode = msg->mode;
        switch (this->mode)
        {
        case CAMMODE_ARMOR:
        {
            this->setupCameraForAimbotMode();
            break;
        }
        case CAMMODE_DYNAMIC_OUTPOST:
        case CAMMODE_FIX_OUTPOST:
        {
            this->setupCameraForHitOutpostMode();
            break;
        }
        case CAMMODE_SMALLRUNE:
        case CAMMODE_BIGRUNE:
        {
            this->setupCameraForRuneMode();
            break;
        }
        case CAMMODE_OUTPOST_F:
        {
            this->setupCameraForOutpostF();
            break;
        }
        default:
        {
            RCLCPP_INFO(this->get_logger(), "未知模式%d", this->mode);
            break;
        }
        }
        RCLCPP_INFO(this->get_logger(), "模式变为 %d", this->mode);
    }
    if (this->color != msg->enemy_color)
    {
        this->color = msg->enemy_color;
        RCLCPP_INFO(this->get_logger(), "颜色设置为 %d", this->color);
    }
}

void ImagePublisher::timer_callback()
{
    sensor_msgs::msg::Image::SharedPtr msg_ptr;
    cv::Mat img;
    int tempmode = mode;

    if (tempmode == CAMMODE_SMALLRUNE || tempmode == CAMMODE_BIGRUNE)
    {
        this->RuneAdjustCameraParam();
    }

    if (!use_video)
        this->get_img(img);
    else
    {
        capture >> img;
        if (img.empty())
            throw CameraException("VIDEO END");
    }
    // if (this->mode == CAMMODE_SMALLRUNE || this->mode == CAMMODE_BIGRUNE)
    // {
    //     cv::circle(img,cv::Point(frame_width/2,frame_height/2),5,cv::Scalar(0,0,255));
    // }

    msg_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
    sensor_msgs::msg::Image msg = *msg_ptr.get();
    msg.header.frame_id = std::to_string(tempmode);
    msg.header.stamp = this->get_clock()->now();

    img_publisher_->publish(msg);
}

ImagePublisher::ImagePublisher(std::string name) : Node(name)
{
    this->init();
    if (this->open())
    {
        img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image", 1);
        readpack_subscribe_ = this->create_subscription<interfaces::msg::Readpack>("Readpack", 1, std::bind(&ImagePublisher::readpack_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::nanoseconds(1), std::bind(&ImagePublisher::timer_callback, this));
    }
}

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    auto node = std::make_shared<ImagePublisher>("node_camdrv");

    RCLCPP_INFO(node->get_logger(), "取图节点已经启动.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
