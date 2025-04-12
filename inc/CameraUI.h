#ifndef CAMERA_UI_H
#define CAMERA_UI_H

#include "SimpleCamera.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <functional>

class CameraUI {
private:
    SimpleCamera& camera;
    std::string windowName;
    bool running;
    
    // 回调函数和跟踪条参数
    static void onExposureChange(int pos, void* userdata);
    static void onGainChange(int pos, void* userdata);
    static void onBrightnessChange(int pos, void* userdata);
    static void onGammaChange(int pos, void* userdata);
    static void onContrastChange(int pos, void* userdata);
    static void onSaturationChange(int pos, void* userdata);
    static void onSharpnessChange(int pos, void* userdata);
    static void onRGainChange(int pos, void* userdata);
    static void onGGainChange(int pos, void* userdata);
    static void onBGainChange(int pos, void* userdata);
    static void onAutoExposureChange(int pos, void* userdata);
    static void onAutoWBChange(int pos, void* userdata);
    static void onSaveConfigButtonClick(int state, void* userdata);
    
    void createTrackbars();
    void createButtons();
    void updateTrackbarsFromCamera();
    
public:
    CameraUI(SimpleCamera& cam, const std::string& name = "Camera Control");
    ~CameraUI();
    
    void init();
    bool run();
    void stop();
};

#endif // CAMERA_UI_H