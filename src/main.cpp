#include "SimpleCamera.h"
#include "CameraUI.h"
#include <iostream>

int main() {
    // 创建相机实例
    SimpleCamera camera;
    
    // 初始化相机
    if (!camera.init()) {
        std::cerr << "相机初始化失败！" << std::endl;
        return -1;
    }
    
    // 创建并初始化UI
    CameraUI ui(camera, "相机控制");
    ui.init();
    
    // 运行UI主循环
    ui.run();
    
    // 关闭相机
    camera.close();
    
    return 0;
}