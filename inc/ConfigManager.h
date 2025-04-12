#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <map>

class ConfigManager {
private:
    std::string configPath;
    
    // 不同类型的配置值映射
    std::map<std::string, int> intValues;
    std::map<std::string, double> doubleValues;
    std::map<std::string, bool> boolValues;
    std::map<std::string, std::string> stringValues;
    
    // 私有方法
    std::string getCurrentTimeString();
    bool createDefaultConfig();
    bool loadConfigValues();
    void setDefaultValues();
    
    // 字符串处理辅助函数
    std::string trim(const std::string& str);

public:
    ConfigManager(const std::string& path = "config/camera_config_info.yaml");
    ~ConfigManager();
    
    // 读取配置文件
    bool loadConfig();
    
    // 保存配置文件
    bool saveConfig();
    
    // 显式关闭配置文件，确保数据写入
    void closeConfigFile();
    
    // 读取参数
    bool readInt(const std::string& key, int& value);
    bool readDouble(const std::string& key, double& value);
    bool readBool(const std::string& key, bool& value);
    
    // 写入参数
    void writeInt(const std::string& key, int value);
    void writeDouble(const std::string& key, double value);
    void writeBool(const std::string& key, bool value);
};

#endif // CONFIG_MANAGER_H