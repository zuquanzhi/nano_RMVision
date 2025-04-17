#ifndef CAMERA_UI_H
#define CAMERA_UI_H

#include "SimpleCamera.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <functional>
#include <ArmorDetect.hpp>

class CameraUI {
private:
    SimpleCamera& camera;
    std::string windowName;
    bool running;
    ArmorDetector* armorDetector;  // 新增：指向主循环中ArmorDetector实例的指针
    
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
    
    // 角点优化参数回调函数
    static void onCornerOptimizationChange(int pos, void* userdata);
    static void onROIExpandRatioChange(int pos, void* userdata);
    static void onLengthExtendFactorChange(int pos, void* userdata);
    static void onMinContourPointsChange(int pos, void* userdata);
    static void onMaxDeviationRatioChange(int pos, void* userdata);
    static void onMinLightbarRatioChange(int pos, void* userdata);
    static void onBrightnessPercentileChange(int pos, void* userdata);
    static void onColorWeightChange(int pos, void* userdata);
    static void onDebugModeChange(int pos, void* userdata);
    
    // 角点验证参数回调函数
    static void onMinAspectRatioChange(int pos, void* userdata);
    static void onMaxAspectRatioChange(int pos, void* userdata);
    static void onParallelToleranceDegChange(int pos, void* userdata);
    static void onMinEdgeRatioChange(int pos, void* userdata);
    static void onMaxEdgeRatioChange(int pos, void* userdata);
    static void onMinDiagRatioChange(int pos, void* userdata);
    static void onMaxDiagRatioChange(int pos, void* userdata);
    static void onMinAreaRatioChange(int pos, void* userdata);
    static void onMaxAreaRatioChange(int pos, void* userdata);
    
    void createTrackbars();
    void createButtons();
    void updateTrackbarsFromCamera();
    
public:
    CameraUI(SimpleCamera& cam, const std::string& name = "Camera Control");
    ~CameraUI();
    
    void init();
    bool run();
    void stop();
    
    // 设置ArmorDetector实例指针
    void setArmorDetector(ArmorDetector* detector) { armorDetector = detector; }
};

#endif // CAMERA_UI_H