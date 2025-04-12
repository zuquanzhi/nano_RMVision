#include "ConfigManager.h"
#include <iostream>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#ifdef __linux__
#include <sys/stat.h> // 添加对chmod的支持
#endif

ConfigManager::ConfigManager(const std::string& path)
    : configPath(path) {
    // 检查路径是否为相对路径，如果是则转换为绝对路径
    if (!path.empty() && path[0] != '/') {
        // 获取当前可执行文件所在目录
        std::filesystem::path execPath = std::filesystem::current_path();
        // 回到项目根目录（假设从build目录运行）
        std::filesystem::path projectRoot = execPath;
        if (execPath.filename() == "build") {
            projectRoot = execPath.parent_path();
        }
        // 构造配置文件的绝对路径
        configPath = (projectRoot / path).string();
    }
    std::cout << "配置文件路径: " << configPath << std::endl;
    
    // 确保配置文件目录存在
    std::filesystem::path configDir = std::filesystem::path(configPath).parent_path();
    if (!std::filesystem::exists(configDir)) {
        std::cout << "创建配置文件目录: " << configDir.string() << std::endl;
        std::filesystem::create_directories(configDir);
    }
    
    // 载入参数
    if (!loadConfigValues()) {
        std::cout << "载入配置文件失败，使用默认值" << std::endl;
        // 设置默认值
        setDefaultValues();
    }
}

// 设置默认值
void ConfigManager::setDefaultValues() {
    // 相机基本状态
    intValues["camera_status"] = 1;
    
    // 分辨率设置
    intValues["resolution_width"] = 1280;
    intValues["resolution_height"] = 1024;
    
    // 曝光控制
    boolValues["auto_exposure"] = true;
    doubleValues["exposure_time"] = 10000.0;
    doubleValues["analog_gain"] = 1.0;
    intValues["brightness"] = 0;
    
    // 图像处理参数
    intValues["gamma"] = 100;
    intValues["contrast"] = 6;
    intValues["saturation"] = 80;
    intValues["sharpness"] = 20;
    
    // 白平衡参数
    boolValues["auto_white_balance"] = false;
    intValues["r_gain"] = 80;
    intValues["g_gain"] = 76;
    intValues["b_gain"] = 110;
}

// 获取当前时间字符串
std::string ConfigManager::getCurrentTimeString() {
    time_t rawtime;
    time(&rawtime);
    char buffer[80];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", localtime(&rawtime));
    return std::string(buffer);
}

ConfigManager::~ConfigManager() {
    // 析构函数不需要特殊处理
}

// 从文件加载配置 - 使用简单的键值对解析
bool ConfigManager::loadConfigValues() {
    try {
        // 如果文件不存在或为空，创建默认配置文件
        if (!std::filesystem::exists(configPath) || 
            std::filesystem::file_size(configPath) == 0) {
            std::cout << "配置文件不存在或为空，创建默认配置" << std::endl;
            createDefaultConfig();
            return true;
        }
        
        // 使用标准文件流读取
        std::ifstream file(configPath);
        if (!file.is_open()) {
            std::cerr << "无法打开配置文件: " << configPath << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // 跳过注释和空行
            if (line.empty() || line[0] == '#') {
                continue;
            }
            
            // 查找键值分隔符
            size_t pos = line.find(':');
            if (pos == std::string::npos) {
                continue;  // 没有冒号，跳过该行
            }
            
            // 提取键和值
            std::string key = line.substr(0, pos);
            std::string valueStr = line.substr(pos + 1);
            
            // 修整键和值的首尾空白字符
            key = trim(key);
            valueStr = trim(valueStr);
            
            // 移除值中的注释部分
            size_t commentPos = valueStr.find('#');
            if (commentPos != std::string::npos) {
                valueStr = valueStr.substr(0, commentPos);
                valueStr = trim(valueStr);
            }
            
            // 尝试将值解析为不同类型并存储
            if (valueStr == "0" || valueStr == "1") {
                // 可能是布尔值
                boolValues[key] = (valueStr == "1");
                intValues[key] = std::stoi(valueStr);
            } else if (valueStr.find('.') != std::string::npos) {
                // 可能是浮点数
                try {
                    double dblVal = std::stod(valueStr);
                    doubleValues[key] = dblVal;
                } catch (...) {
                    // 转换失败，当作字符串处理
                    stringValues[key] = valueStr;
                }
            } else {
                // 尝试转换为整数
                try {
                    int intVal = std::stoi(valueStr);
                    intValues[key] = intVal;
                } catch (...) {
                    // 转换失败，当作字符串处理
                    stringValues[key] = valueStr;
                }
            }
        }
        
        file.close();
        std::cout << "成功从文件加载配置: " << configPath << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "读取配置时出错: " << e.what() << std::endl;
        return false;
    }
}

// 修整字符串首尾的空白字符
std::string ConfigManager::trim(const std::string& str) {
    const auto strBegin = str.find_first_not_of(" \t\r\n");
    if (strBegin == std::string::npos)
        return ""; // 全是空白字符
    
    const auto strEnd = str.find_last_not_of(" \t\r\n");
    const auto strRange = strEnd - strBegin + 1;
    
    return str.substr(strBegin, strRange);
}

// 创建默认配置文件 - 直接写入文件
bool ConfigManager::createDefaultConfig() {
    try {
        // 设置默认值
        setDefaultValues();
        
        // 保存到文件
        return saveConfig();
    }
    catch (const std::exception& e) {
        std::cerr << "创建默认配置文件时出错: " << e.what() << std::endl;
        return false;
    }
}

bool ConfigManager::loadConfig() {
    return loadConfigValues();
}

// 直接使用标准C++文件操作保存配置
bool ConfigManager::saveConfig() {
    try {
        // 创建备份 - 使用时间戳命名，避免覆盖旧备份
        if (std::filesystem::exists(configPath)) {
            // 获取当前时间作为备份文件的后缀
            time_t now = time(nullptr);
            char timestamp[20];
            strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", localtime(&now));
            
            // 构造备份文件名: 原文件名.时间戳.bak
            std::string backupPath = configPath + "." + timestamp + ".bak";
            
            try {
                std::filesystem::copy_file(configPath, backupPath);
                std::cout << "已创建备份: " << backupPath << std::endl;
            } catch (const std::exception& e) {
                std::cerr << "创建备份文件失败: " << e.what() << std::endl;
                // 继续保存，不因备份失败而中断保存过程
            }
        }
        
        // 打开文件进行写入
        std::ofstream file(configPath);
        if (!file.is_open()) {
            std::cerr << "无法打开配置文件进行写入: " << configPath << std::endl;
            return false;
        }
        
        // 写入文件头
        file << "# 相机配置文件 - 保存于 " << getCurrentTimeString() << "\n\n";
        
        // 写入所有参数，按类别分组
        
        // 基本状态
        file << "# 相机基本状态\n";
        file << "camera_status: " << intValues["camera_status"] << "\n\n";
        
        // 分辨率设置
        file << "# 分辨率设置\n";
        file << "resolution_width: " << intValues["resolution_width"] << "\n";
        file << "resolution_height: " << intValues["resolution_height"] << "\n\n";
        
        // 曝光控制
        file << "# 曝光控制\n";
        file << "auto_exposure: " << (boolValues["auto_exposure"] ? 1 : 0) 
             << "             # 自动曝光 (1=开启, 0=关闭)\n";
        file << "exposure_time: " << std::fixed << std::setprecision(1) 
             << doubleValues["exposure_time"] 
             << "       # 曝光时间 (微秒)\n";
        file << "analog_gain: " << std::fixed << std::setprecision(1) 
             << doubleValues["analog_gain"] 
             << "             # 模拟增益 (1-16)\n";
        file << "brightness: " << intValues["brightness"] 
             << "                # 目标亮度 (0-1000)\n\n";
        
        // 图像处理参数
        file << "# 图像处理参数\n";
        file << "gamma: " << intValues["gamma"] 
             << "                   # Gamma值 (0-350)\n";
        file << "contrast: " << intValues["contrast"] 
             << "                  # 对比度 (-50-100)\n";
        file << "saturation: " << intValues["saturation"] 
             << "               # 饱和度 (0-200)\n";
        file << "sharpness: " << intValues["sharpness"] 
             << "                # 锐度 (0-100)\n\n";
        
        // 白平衡参数
        file << "# 白平衡参数\n";
        file << "auto_white_balance: " << (boolValues["auto_white_balance"] ? 1 : 0) 
             << "        # 自动白平衡 (1=开启, 0=关闭)\n";
        file << "r_gain: " << intValues["r_gain"] 
             << "                   # 红色增益 (0-400)\n";
        file << "g_gain: " << intValues["g_gain"] 
             << "                   # 绿色增益 (0-400)\n";
        file << "b_gain: " << intValues["b_gain"] 
             << "                  # 蓝色增益 (0-400)\n";
        
        file.close();
        
        // 设置文件权限
        #ifdef __linux__
        chmod(configPath.c_str(), 0666); // 所有用户可读写
        #endif
        
        std::cout << "成功保存配置到 " << configPath << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "保存配置文件时出错: " << e.what() << std::endl;
        return false;
    }
}

// 读取参数 - 整数
bool ConfigManager::readInt(const std::string& key, int& value) {
    try {
        auto it = intValues.find(key);
        if (it != intValues.end()) {
            value = it->second;
            return true;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "读取整数键 " << key << " 时出错: " << e.what() << std::endl;
    }
    return false;
}

// 读取参数 - 浮点数
bool ConfigManager::readDouble(const std::string& key, double& value) {
    try {
        auto it = doubleValues.find(key);
        if (it != doubleValues.end()) {
            value = it->second;
            return true;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "读取浮点数键 " << key << " 时出错: " << e.what() << std::endl;
    }
    return false;
}

// 读取参数 - 布尔值
bool ConfigManager::readBool(const std::string& key, bool& value) {
    try {
        auto it = boolValues.find(key);
        if (it != boolValues.end()) {
            value = it->second;
            return true;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "读取布尔键 " << key << " 时出错: " << e.what() << std::endl;
    }
    return false;
}

// 写入参数 - 整数
void ConfigManager::writeInt(const std::string& key, int value) {
    intValues[key] = value;
}

// 写入参数 - 浮点数
void ConfigManager::writeDouble(const std::string& key, double value) {
    doubleValues[key] = value;
}

// 写入参数 - 布尔值
void ConfigManager::writeBool(const std::string& key, bool value) {
    boolValues[key] = value;
}

// 关闭配置文件
void ConfigManager::closeConfigFile() {
    // 保存所有内存中的值到文件
    saveConfig();
}