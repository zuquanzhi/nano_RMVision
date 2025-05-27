#ifndef CAMERA_UI_H
#define CAMERA_UI_H

#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <functional>

class CameraUI {
private:
    SimpleCamera& camera;
    std::string windowName;
    bool running;
    
    // 回调函数和跟踪条参数
    static void onExposureChange(int value, void* userdata);
    static void onGainChange(int value, void* userdata);
    static void onGammaChange(int value, void* userdata);
    static void onContrastChange(int value, void* userdata);
    static void onSaturationChange(int value, void* userdata);
    static void onSharpnessChange(int value, void* userdata);
    static void onWBRChange(int value, void* userdata);
    static void onWBGChange(int value, void* userdata);
    static void onWBBChange(int value, void* userdata);
    static void onAutoExposureOn(int state, void* userdata);
    static void onAutoExposureOff(int state, void* userdata);
    static void onAutoWBOn(int state, void* userdata);
    static void onAutoWBOff(int state, void* userdata);
    static void onSaveConfigButtonClick(int state, void* userdata);

    // New trackbar callbacks for armor detection thresholds
    static void onArmorBrightnessThresholdRedChange(int value, void* userdata);
    static void onArmorBrightnessThresholdBlueChange(int value, void* userdata);
    static void onBrightnessThresholdChange(int value, void* userdata);
    static void onChannalSumThresholdChange(int value, void* userdata);


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