#include "SimpleCamera.h"
#include "CameraUI.h"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <ArmorDetect.hpp>

// 定义命令行帮助信息
void printHelp() {
    std::cout << "相机控制程序帮助：" << std::endl;
    std::cout << "用法: simple_camera [选项]" << std::endl;
    std::cout << "选项:" << std::endl;
    std::cout << "  --help, -h       显示帮助信息" << std::endl;
    std::cout << "  --no-gui, -n     无GUI模式运行，仅相机捕获" << std::endl;
    std::cout << "  --headless       无GUI模式运行，与--no-gui相同" << std::endl;
    std::cout << "  --display-env    显示DISPLAY环境变量设置" << std::endl;
    std::cout << "  --force-x11      强制使用X11后端显示，在SSH -X连接时可能有帮助" << std::endl;
    std::cout << "  --fix-black      启用黑框修复模式，解决SSH -X连接下出现黑框问题" << std::endl;
}

// 尝试设置X11相关环境变量
void setupX11Environment() {
    // 打印当前X11环境信息
    const char* display = std::getenv("DISPLAY");
    std::cout << "当前DISPLAY环境变量: " << (display ? display : "未设置") << std::endl;
    
    // 如果DISPLAY未设置，尝试设置一个默认值
    if (!display || strlen(display) == 0) {
        std::cout << "DISPLAY未设置，尝试设置为:0.0" << std::endl;
        setenv("DISPLAY", ":0.0", 0);
    }
    
    // 设置OpenCV GUI相关环境变量
    setenv("OPENCV_VIDEOIO_PRIORITY_LIST", "!GTK", 1);
    
    // 检查QT后端是否可用，并优先使用
    #if CV_MAJOR_VERSION >= 4
    setenv("OPENCV_GUI_BACKEND", "QT", 0);
    #endif
    
    // 禁用OpenCV的自动后端选择
    setenv("OPENCV_DISABLE_AUTO_BACKEND_SELECTION", "1", 1);
    
    // 禁用空指针检查
    setenv("QT_FATAL_WARNINGS", "0", 1);
}

int main(int argc, char* argv[]) {
    bool useGUI = true;
    bool forceX11 = false;
    
    // 解析命令行参数
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        } else if (arg == "--no-gui" || arg == "-n" || arg == "--headless") {
            useGUI = false;
            std::cout << "运行在无GUI模式" << std::endl;
        } else if (arg == "--display-env") {
            const char* display = std::getenv("DISPLAY");
            std::cout << "DISPLAY环境变量: " << (display ? display : "未设置") << std::endl;
            
            // 显示X11相关的环境变量
            const char* xauth = std::getenv("XAUTHORITY");
            std::cout << "XAUTHORITY环境变量: " << (xauth ? xauth : "未设置") << std::endl;
        } else if (arg == "--force-x11") {
            forceX11 = true;
            std::cout << "强制使用X11显示" << std::endl;
        } else if (arg == "--fix-black") {
            // 添加特殊参数，尝试修复黑框问题
            std::cout << "启用黑框修复模式" << std::endl;
            setenv("OPENCV_WINDOW_DECORATIONS", "0", 1);
            setenv("OPENCV_GUI_BACKEND", "GTK3", 0);
        }
    }
    
    // 设置X11环境
    setupX11Environment();
    
    // 创建相机实例
    SimpleCamera camera;
    
    // 初始化相机
    if (!camera.init()) {
        std::cerr << "相机初始化失败！" << std::endl;
        return -1;
    }
    
    if (useGUI) {
        try {
            std::cout << "尝试创建GUI界面..." << std::endl;
            
            // 强制使用X11或指定的后端
            if (forceX11) {
                std::cout << "强制使用X11后端" << std::endl;
                // 验证X11连接
                cv::namedWindow("X11测试", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
                // 创建测试图像以验证显示
                cv::Mat testImage(300, 400, CV_8UC3, cv::Scalar(200, 200, 200));
                cv::putText(testImage, "X11测试 - 按任意键继续", cv::Point(50, 150), 
                           cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
                cv::imshow("X11测试", testImage);
                cv::waitKey(1000); // 显示1秒
                cv::destroyWindow("X11测试");
            }
            
            // 创建并初始化UI
            CameraUI ui(camera, "相机控制");
            ui.init();
            
            // 初始化装甲板检测器
            std::string model_path = std::filesystem::absolute("../model/last.xml").string();
            ArmorDetector armorDetector(model_path);
            armorDetector.init();
            
            std::cout << "UI界面初始化完成" << std::endl;
            
            // 运行UI主循环
            std::cout << "启动UI主循环，按'r'键可刷新解决黑框问题" << std::endl;
            ui.run();
        }
        catch (const cv::Exception& e) {
            std::cerr << "OpenCV异常: " << e.what() << std::endl;
            std::cerr << "无法启动图形界面，可能原因:" << std::endl;
            std::cerr << "1. X11转发未正确设置" << std::endl;
            std::cerr << "2. 远程主机没有GTK/QT后端支持" << std::endl;
            std::cerr << "3. SSH -X连接不稳定" << std::endl;
            std::cerr << "尝试解决方案:" << std::endl;
            std::cerr << "- 使用 '--fix-black' 参数启用黑框修复" << std::endl;
            std::cerr << "- 使用 'ssh -Y' 而不是 'ssh -X'" << std::endl;
            std::cerr << "- 在主机上运行 'export XLIB_SKIP_ARGB_VISUALS=1'" << std::endl;
            std::cerr << "切换到无GUI模式..." << std::endl;
            useGUI = false;
        }
    }
    
    // 无GUI模式
    if (!useGUI) {
        std::cout << "运行在无GUI模式。按Ctrl+C退出程序。" << std::endl;
        
        // 在无GUI模式下，相机已经在init()中初始化，
        // 但我们可能需要继续处理图像
        cv::Mat frame;
        
        // 保持程序运行
        while (true) {
            // 捕获一帧图像（可以在这里添加处理代码）
            if (camera.getFrame(frame)) {
                // 可以在这里添加图像处理代码
                // 例如：保存图像、分析图像等
            }
            
            // 休眠100毫秒
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    // 关闭相机
    camera.close();
    
    return 0;
}