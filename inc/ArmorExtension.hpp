#ifndef ARMOR_EXTENSION_HPP
#define ARMOR_EXTENSION_HPP

#include <opencv2/opencv.hpp>
#include <array>
#include "infer.h"

// 扩展Armor结构体为类
class ArmorExt {
public:
    // 角点坐标数组
    std::array<float, 4> x;
    std::array<float, 4> y;
    float score;
    int label;
    
    // 从基础Armor构造函数
    ArmorExt(const Armor& base) {
        x = {base.x1, base.x2, base.x3, base.x4};
        y = {base.y1, base.y2, base.y3, base.y4};
        score = base.score;
        label = base.label;
    }
    
    // 默认构造函数
    ArmorExt() {
        x = {0.0f, 0.0f, 0.0f, 0.0f};
        y = {0.0f, 0.0f, 0.0f, 0.0f};
        score = 0.0f;
        label = 0;
    }
    
    // 转换回基础Armor
    Armor toArmor() const {
        Armor result;
        result.x1 = x[0];
        result.y1 = y[0];
        result.x2 = x[1];
        result.y2 = y[1];
        result.x3 = x[2];
        result.y3 = y[2];
        result.x4 = x[3];
        result.y4 = y[3];
        result.score = score;
        result.label = label;
        return result;
    }
    
    // 角点修正函数
    bool correct_promote(cv::Mat &src , const float ROI_EXPAND_RATIO = 0.6f,
                         const float LENGTH_EXTEND_FACTOR = 1.7f,
                         const int MIN_CONTOUR_POINTS = 6,
                         const float MAX_DEVIATION_RATIO = 0.95f,
                         const float MIN_LIGHTBAR_RATIO = 2.0f,
                         const float BRIGHTNESS_PERCENTILE = 0.05f,
                         const float COLOR_WEIGHT = 0.1f,
                         const bool debug = false,
                         // 角点验证参数 - 已根据实际装甲板特性调整
                         const float MIN_ASPECT_RATIO = 2.0f,         // 最小宽高比，适应实际约2.8的比例
                         const float MAX_ASPECT_RATIO = 5.0f,          
                         const float PARALLEL_TOLERANCE_DEG = 8.0f,    // 平行度容差(度)，放宽容差
                         const float MIN_EDGE_RATIO = 0.69f,           // 最小对边长度比，适应约0.96的实际比例
                         const float MAX_EDGE_RATIO = 1.2f,            
                         const float MIN_DIAG_RATIO = 0.85f,           // 最小对角线比，放宽容差
                         const float MAX_DIAG_RATIO = 1.2f,           
                         const float MIN_AREA_RATIO = 0.7f,            // 最小面积比
                         const float MAX_AREA_RATIO = 1.3f             // 最大面积比
                        ) {
        // 原始坐标备份
        const std::array<cv::Point2f, 4> orig_corners = {
            cv::Point2f(x[0], y[0]), cv::Point2f(x[1], y[1]),
            cv::Point2f(x[2], y[2]), cv::Point2f(x[3], y[3])
        };

        cv::Mat debug_visual;
        if (debug) {
            // 调试可视化
            cv::cvtColor(src, debug_visual, cv::COLOR_BGR2GRAY);
            cv::cvtColor(debug_visual, debug_visual, cv::COLOR_GRAY2BGR);
        }

        // 可视化修正前的角点
        if (debug) {
            for (const auto& pt : orig_corners) {
                cv::circle(debug_visual, pt, 5, cv::Scalar(255, 0, 255), -1); // 紫色圆圈
            }
        }

        // 计算左右灯条向量和角度
        const cv::Vec2f left_vec = orig_corners[1] - orig_corners[0];
        const cv::Vec2f right_vec = orig_corners[2] - orig_corners[3];

        // 计算灯条角度(规范化到[-90,90]度)
        auto calc_lightbar_angle = [](const cv::Vec2f& vec) {
            float angle = std::atan2(vec[1], vec[0]) * 180 / CV_PI;
            return (angle > 90) ? angle - 180 : (angle < -90) ? angle + 180 : angle;
        };

        const float left_angle = calc_lightbar_angle(left_vec);
        const float right_angle = calc_lightbar_angle(right_vec);
        const float avg_height = (cv::norm(left_vec) + cv::norm(right_vec)) / 2;
        const float max_deviation = avg_height * MAX_DEVIATION_RATIO;

        // 创建旋转ROI区域
        auto create_rotated_roi = [&](const cv::Point2f& p1, const cv::Point2f& p2, float angle) {
            const float length = cv::norm(p2 - p1);  // 灯条长度
            const float width = length * ROI_EXPAND_RATIO;  // 灯条宽度
            const cv::Point2f center = (p1 + p2) / 2;

            cv::RotatedRect roi_rect(center,cv::Size2f(length * LENGTH_EXTEND_FACTOR, width),angle);

            std::vector<cv::Point2f> pts(4);
            roi_rect.points(pts.data());
            return cv::boundingRect(pts);
        };

        // 处理灯条的通用函数
        auto process_lightbar = [&](const cv::Point2f& p1, const cv::Point2f& p2,float angle, int idx1, int idx2) {
            if (cv::norm(p2 - p1) == 0) return false;

            // 创建ROI区域
            cv::Rect roi = create_rotated_roi(p1, p2, angle);
            roi &= cv::Rect(0, 0, src.cols, src.rows);
            if (roi.area() == 0) return false;

            // 在调试图像上绘制ROI区域
            if (debug) {
                cv::rectangle(debug_visual, roi, cv::Scalar(255, 0, 0), 2); // 蓝色矩形框
            }

            // 提取ROI区域
            cv::Mat roi_img = src(roi);

            // 基于亮度的二值化
            cv::Mat gray, binary_bright;
            cv::cvtColor(roi_img, gray, cv::COLOR_BGR2GRAY);

            // 使用百分位阈值
            std::vector<uchar> pixels(gray.begin<uchar>(), gray.end<uchar>());
            if (pixels.empty()) return false;
            
            size_t nth = pixels.size() * (1.0f - BRIGHTNESS_PERCENTILE);
            std::nth_element(pixels.begin(), pixels.begin() + nth, pixels.end());
            uchar threshold = pixels[nth];
            cv::threshold(gray, binary_bright, threshold, 255, cv::THRESH_BINARY);


            // 基于红蓝差值的二值化
            std::vector<cv::Mat> channels;
            cv::split(roi_img, channels); // 分离通道
            cv::Mat blue_binary, red_binary, binary_color;
            cv::threshold(channels[0], blue_binary, 128, 255, cv::THRESH_BINARY); // 蓝色二值化
            cv::threshold(channels[2], red_binary, 128, 255, cv::THRESH_BINARY); // 红色二值化
            cv::bitwise_or(blue_binary, red_binary, binary_color); // 合并红蓝二值图

            // 结合两种二值化结果
            cv::Mat binary_combined;
            cv::addWeighted(binary_bright, COLOR_WEIGHT, binary_color, 1 - COLOR_WEIGHT, 0, binary_combined);

            // 轮廓检测
            // std::vector<std::vector<cv::Point>> contours;
            // cv::findContours(binary_combined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);


            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
            cv::morphologyEx(binary_combined, binary_combined, cv::MORPH_OPEN, kernel);

            // 轮廓检测
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary_combined, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            bool found = false;
            for (auto &contour : contours) {
                if (contour.size() < MIN_CONTOUR_POINTS) continue;

                cv::RotatedRect rect = cv::minAreaRect(contour);
                std::vector<cv::Point2f> rect_pts(4);
                rect.points(rect_pts.data());
                for (auto &p : rect_pts) p += cv::Point2f(roi.tl());

                // 调试绘制轮廓矩形框
                if (debug) {
                    for (int i = 0; i < 4; i++) {
                        cv::line(debug_visual, rect_pts[i], rect_pts[(i + 1) % 4], cv::Scalar(0, 0, 255), 2); // 红色矩形框   
                    }
                }

                //计算红色矩形的四边中点
                cv::Point2f mid1 = (rect_pts[0] + rect_pts[1]) / 2;
                cv::Point2f mid2 = (rect_pts[2] + rect_pts[3]) / 2;
                cv::Point2f mid3 = (rect_pts[1] + rect_pts[2]) / 2;
                cv::Point2f mid4 = (rect_pts[0] + rect_pts[3]) / 2;

                float distance1 = cv::norm(mid1 - mid2);
                float distance2 = cv::norm(mid3 - mid4);

                if (distance1 > distance2) {
                    x[idx1] = mid1.x; y[idx1] = mid1.y;  // 更新为四个角点的中点
                    x[idx2] = mid2.x; y[idx2] = mid2.y;  // 更新为四个角点的中点
                }
                else {
                    x[idx1] = mid3.x; y[idx1] = mid3.y;  // 更新为四个角点的中点
                    x[idx2] = mid4.x; y[idx2] = mid4.y;  // 更新为四个角点的中点
                }
                found = true;

                break; // 找到合适的轮廓后退出循环
            }

            return found;
        };

        // 处理左右灯条
        bool left_updated = process_lightbar(orig_corners[0], orig_corners[1], left_angle, 0, 1);
        bool right_updated = process_lightbar(orig_corners[3], orig_corners[2], right_angle, 3, 2);

        // 纠正验证触发
        std::vector<cv::Point2f> new_corners = {{x[0], y[0]}, {x[1], y[1]}, {x[2], y[2]}, {x[3], y[3]}};
        bool valid_corners = true;
        
        if (left_updated && right_updated) {
            // 检查装甲板宽高比
            float width = (cv::norm(new_corners[0] - new_corners[3]) + cv::norm(new_corners[1] - new_corners[2])) / 2;
            float height = (cv::norm(new_corners[0] - new_corners[1]) + cv::norm(new_corners[3] - new_corners[2])) / 2;
            float aspect_ratio = width / height;
            
            // 装甲板宽高比应在合理范围内 (使用可配置参数)
            if (aspect_ratio < MIN_ASPECT_RATIO || aspect_ratio > MAX_ASPECT_RATIO) {
                valid_corners = false;
                if (debug) {
                    std::cout << "装甲板宽高比不合理: " << aspect_ratio << " (允许范围: " 
                              << MIN_ASPECT_RATIO << "-" << MAX_ASPECT_RATIO << ")" << std::endl;
                }
            }
            
            // 2.检查四条边的平行度
            // // 获取四条边的方向向量
            // cv::Vec2f top_dir = new_corners[3] - new_corners[0];
            // cv::Vec2f bottom_dir = new_corners[2] - new_corners[1];
            // cv::Vec2f left_dir = new_corners[1] - new_corners[0];
            // cv::Vec2f right_dir = new_corners[2] - new_corners[3];

            // // 归一化向量
            // top_dir = top_dir / cv::norm(top_dir);
            // bottom_dir = bottom_dir / cv::norm(bottom_dir);
            // left_dir = left_dir / cv::norm(left_dir);
            // right_dir = right_dir / cv::norm(right_dir);

            // // 计算相对方向的平行度分数（绝对值表示平行度，1表示完全平行，0表示垂直）
            // float top_bottom_parallel = std::abs(top_dir.dot(bottom_dir));
            // float left_right_parallel = std::abs(left_dir.dot(right_dir));

            // // 根据平行度容差(度)计算阈值
            // const float MIN_PARALLEL_SCORE = std::cos(PARALLEL_TOLERANCE_DEG * CV_PI / 180.0f);

            // // 验证平行度
            // bool parallel_valid = (top_bottom_parallel >= MIN_PARALLEL_SCORE && 
            //                     left_right_parallel >= MIN_PARALLEL_SCORE);

            // if (!parallel_valid) {
            //     valid_corners = false;
            //     if (debug) {
            //         std::cout << "边缘平行度检查失败: 上下平行度=" << top_bottom_parallel 
            //                 << ", 左右平行度=" << left_right_parallel
            //                 << " (阈值: " << MIN_PARALLEL_SCORE << ", 容差: " 
            //                 << PARALLEL_TOLERANCE_DEG << "度)" << std::endl;
            //     }
            // }
            
            // 3. 检查四边形是否接近平行四边形
            float top_len = cv::norm(new_corners[0] - new_corners[3]);
            float bottom_len = cv::norm(new_corners[1] - new_corners[2]);
            float left_len = cv::norm(new_corners[0] - new_corners[1]);
            float right_len = cv::norm(new_corners[3] - new_corners[2]);
            
            // 检查对边长度比例
            float top_bottom_ratio = top_len / bottom_len;
            float left_right_ratio = left_len / right_len;
            
            if (top_bottom_ratio < MIN_EDGE_RATIO || top_bottom_ratio > MAX_EDGE_RATIO ||
                left_right_ratio < MIN_EDGE_RATIO || left_right_ratio > MAX_EDGE_RATIO) {
                valid_corners = false;
                if (debug) {
                    std::cout << "装甲板对边长度不匹配: " << "上下比 = " << top_bottom_ratio 
                              << ", 左右比 = " << left_right_ratio 
                              << " (允许范围: " << MIN_EDGE_RATIO << "-" << MAX_EDGE_RATIO << ")" << std::endl;
                }
            }
            
            // 4. 检查对角线长度比例
            float diag1 = cv::norm(new_corners[0] - new_corners[2]);
            float diag2 = cv::norm(new_corners[1] - new_corners[3]);
            float diag_ratio = diag1 / diag2;
            
            if (diag_ratio < MIN_DIAG_RATIO || diag_ratio > MAX_DIAG_RATIO) {
                valid_corners = false;
                if (debug) {
                    std::cout << "装甲板对角线长度不匹配: " << diag_ratio 
                              << " (允许范围: " << MIN_DIAG_RATIO << "-" << MAX_DIAG_RATIO << ")" << std::endl;
                }
            }
            
            // 5. 检查与原始装甲板的面积差异
            float orig_area = cv::contourArea(orig_corners);
            float new_area = cv::contourArea(new_corners);
            float area_ratio = new_area / orig_area;
            
            if (area_ratio < MIN_AREA_RATIO || area_ratio > MAX_AREA_RATIO) {
                valid_corners = false;
                if (debug) {
                    std::cout << "装甲板面积变化过大: " << area_ratio 
                              << " (允许范围: " << MIN_AREA_RATIO << "-" << MAX_AREA_RATIO << ")" << std::endl;
                }
            }
            
            // 6. 凸多边形检查
            if (!cv::isContourConvex(new_corners)) {
                valid_corners = false;
                if (debug) {
                    std::cout << "装甲板不是凸四边形" << std::endl;
                }
            }
        } else {
            // 如果有任一灯条未成功更新，则验证失败
            valid_corners = false;
        }
        
        // 如果角点验证失败，设置为无效
        if (!valid_corners) {
            // 设置一个特殊值表示无效角点
            for (int i = 0; i < 4; ++i) {
                x[i] = -1.0f;
                y[i] = -1.0f;
            }
            
            if (debug) {
                // 添加红色半透明遮罩表示无效
                cv::Mat overlay;
                debug_visual.copyTo(overlay);
                cv::rectangle(overlay, cv::Point(0, 0), cv::Point(src.cols, src.rows), 
                            cv::Scalar(0, 0, 255), -1);  // 红色填充矩形
                
                // 应用透明度
                double alpha = 0.3;  // 透明度：0.3表示30%的不透明度
                cv::addWeighted(overlay, alpha, debug_visual, 1 - alpha, 0, debug_visual);
                
                // 添加明显的失效标记
                cv::rectangle(debug_visual, cv::Point(10, 10), cv::Point(180, 70), 
                            cv::Scalar(0, 0, 180), -1);  // 深红色背景
                            
                cv::putText(debug_visual, "ARMOR INVALID", cv::Point(20, 40), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);
                cv::putText(debug_visual, "Corner Validation Failed", cv::Point(20, 60), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                
                // 在装甲板位置绘制红色十字
                cv::Point2f center(0, 0);
                for (const auto& pt : orig_corners) {
                    center += pt;
                }
                center *= 0.25f;  // 计算中心点
                
                int cross_size = 15;
                cv::line(debug_visual, 
                        cv::Point(center.x - cross_size, center.y - cross_size),
                        cv::Point(center.x + cross_size, center.y + cross_size),
                        cv::Scalar(0, 0, 255), 2);
                cv::line(debug_visual, 
                        cv::Point(center.x + cross_size, center.y - cross_size),
                        cv::Point(center.x - cross_size, center.y + cross_size),
                        cv::Scalar(0, 0, 255), 2);
                
                // 显示调试结果
                cv::imshow("Corner Correction Debug", debug_visual);
                cv::waitKey(1);
            }
            return false;
        }

        // 可视化修正后的角点（绿色）
        if (debug) {
            for (const auto& pt : new_corners) {
                cv::circle(debug_visual, pt, 5, cv::Scalar(0, 255, 0), -1); // 绿色圆圈
            }

            // 显示调试结果
            cv::imshow("Corner Correction Debug", debug_visual);
            cv::waitKey(1);
        }
        
        // 返回修正结果
        return true;
    }
};

#endif // ARMOR_EXTENSION_HPP
