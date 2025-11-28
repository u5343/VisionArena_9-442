#include <cv_bridge/cv_bridge.h>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <referee_pkg/msg/multi_object.hpp>
#include <referee_pkg/msg/object.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "yolo_detect.hpp"
#include <std_msgs/msg/header.hpp>

#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class LocateNode : public rclcpp::Node
{
public:
    LocateNode(string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing TestNode");
        Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            bind(&LocateNode::callback_camera, this, std::placeholders::_1));
        Target_pub = this->create_publisher<referee_pkg::msg::MultiObject>(
            "/vision/target", 10);
        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
        detector_ = std::make_shared<YOLODetector>();

        RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
    }
    ~LocateNode() { cv::destroyWindow("Detection Result"); }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;
    std::shared_ptr<YOLODetector> detector_;

    std::vector<std::vector<cv::Point2f>> rectangle_corners_list; // 矩形角点列表
    std::vector<std::vector<cv::Point2f>> sphere_corners_list;    // 球体角点列表
    std::vector<std::vector<cv::Point2f>> midpoint_corners_list;
    std::vector<std::vector<cv::Point2f>> armor_corners_list;

    // 对应的目标类型
    std::vector<std::string> armor_types;     // 装甲板类型
    std::vector<std::string> rectangle_types; // 矩形类型
    std::vector<std::string> sphere_types;    // 球体类型
    vector<Mat> number_templates;
    struct TrackedObject
    {
        int id;
        int number;     // 识别到的数字
        Point2f center; // 中心位置
    };

    vector<TrackedObject> tracked_objects; // 多个目标列表
    int next_id = 1;                       // 下一个可用的ID

    vector<Point2f> calculateStableSpherePoints(const Point2f &center,
                                                float radius)
    {
        vector<Point2f> points;

        // 简单稳定的几何计算，避免漂移
        // 左、下、右、上
        points.push_back(Point2f(center.x - radius, center.y)); // 左点 (1)
        points.push_back(Point2f(center.x, center.y + radius)); // 下点 (2)
        points.push_back(Point2f(center.x + radius, center.y)); // 右点 (3)
        points.push_back(Point2f(center.x, center.y - radius)); // 上点 (4)

        return points;
    }

    vector<Point2f> sortCorners(const vector<Point2f> &corners)
    {
        if (corners.size() != 4)
            return corners;

        vector<Point2f> sorted(4);

        // 计算中心点
        Point2f center(0, 0);
        for (const auto &p : corners)
            center += p;
        center /= 4.0f;

        // 分类四个角点
        for (const auto &p : corners)
        {
            if (p.x < center.x && p.y > center.y)
                sorted[0] = p; // 左下
            else if (p.x < center.x && p.y < center.y)
                sorted[1] = p; // 左上
            else if (p.x > center.x && p.y < center.y)
                sorted[2] = p; // 右上
            else if (p.x > center.x && p.y > center.y)
                sorted[3] = p; // 右下
        }

        return sorted;
    }
    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg)
    {

        armor_types.clear();
        rectangle_corners_list.clear();
        rectangle_types.clear();
        sphere_corners_list.clear();
        sphere_types.clear();
        armor_corners_list.clear();

        // 图像转换
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat image = cv_ptr->image;
        // Mat image2 =cv_ptr->image;
        Mat hsv, mask;
        cvtColor(image, hsv, cv::COLOR_BGR2HSV);
        inRange(hsv, Scalar(0, 0, 0), Scalar(180, 50, 50), mask);
        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<vector<Point>> conpoly(contours.size());
        for (size_t i = 0; i < contours.size(); i++)
        {
            double area = cv::contourArea(contours[i]);
            if (area > 500)
            {
                vector<Point> hull;
                convexHull(contours[i], hull); // 计算凸包

                double peri = arcLength(hull, true);
                vector<Point> approx;
                approxPolyDP(hull, approx, 0.02 * peri, true); // 对凸包进行多边形逼近

                if (approx.size() == 4 || approx.size() == 5 || approx.size() == 6)
                {
                    // 绘制凸包轮廓
                    vector<vector<Point>> hull_contours = {approx};
                    // drawContours(image, hull_contours, 0, Scalar(0, 255, 255), 3);

                    // 使用最小外接矩形找4个有序角点
                    RotatedRect rect = minAreaRect(approx); // 最小外接矩形

                    Point2f rect_points[4];
                    rect.points(rect_points); // 直接得到4个有序角点

                    // 绘制矩形角点（用绿色标记，区别于蓝色原始角点）
                    // for (int j = 0; j < 4; j++)
                    // {
                    //     circle(image, rect_points[j], 8, Scalar(0, 255, 0), 1);
                    //     // line(image, rect_points[j], rect_points[(j + 1) % 4], Scalar(0, 255, 0), 2);
                    // }

                    // 修改为多目标跟踪逻辑
                    Point2f current_center = rect.center;
                    int current_id = -1;
                    bool is_existing_target = false;

                    // 查找最近的已有目标
                    float min_distance = 50.0f;
                    int best_match_index = -1;

                    for (int i = 0; i < tracked_objects.size(); i++)
                    {
                        float distance = norm(current_center - tracked_objects[i].center);
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            best_match_index = i;
                            is_existing_target = true;
                        }
                    }

                    if (is_existing_target)
                    {
                        // 更新现有目标
                        current_id = tracked_objects[best_match_index].id;
                        tracked_objects[best_match_index].center = current_center;
                    }
                    else
                    {
                        // 创建新目标
                        current_id = next_id++;
                        tracked_objects.push_back({current_id, -1, current_center});
                    }

                    vector<Point2f> perspective_points;
                    for (int j = 0; j < 4; j++)
                    {
                        perspective_points.push_back(rect_points[j]);
                    }

                    perspective_points = sortCorners(perspective_points);

                    // 调用透视函数
                    // Mat corrected_armor = perspectiveCorrection(image, perspective_points);

                    // 多目标数字识别
                    if (!image.empty())
                    {
                        int current_number = detector_->detect(image);
                        RCLCPP_INFO(this->get_logger(), "YOLO:%d", current_number);
                        if (current_number != -1)
                        {
                            // 更新对应目标的数字
                            for (auto &obj : tracked_objects)
                            {
                                if (obj.id == current_id)
                                {
                                    obj.number = current_number;

                                    // 在这里添加装甲板类型存储
                                    std::string type_str = "armor_" + std::to_string(current_number);

                                    std::vector<cv::Point2f> armor_corners;
                                    for (int j = 0; j < 4; j++)
                                    {
                                        armor_corners.push_back(perspective_points[j]);
                                    }

                                    armor_types.push_back(type_str);
                                    break;
                                }
                            }
                        }
                        else
                        {
                            // 识别失败，也存储默认装甲板类型
                            std::vector<cv::Point2f> armor_corners;
                            for (int j = 0; j < 4; j++)
                            {
                                armor_corners.push_back(perspective_points[j]);
                            }

                            armor_types.push_back("armor");
                        }
                    }
                    else
                    {
                        // 如果没有透视变换图像，也存储默认装甲板类型 ↓
                        std::vector<cv::Point2f> armor_corners;
                        for (int j = 0; j < 4; j++)
                        {
                            armor_corners.push_back(perspective_points[j]);
                        }

                        armor_types.push_back("armor");
                    }

                    // 显示目标信息（在矩形右上角）
                    for (const auto &obj : tracked_objects)
                    {
                        if (obj.id == current_id)
                        {
                            string display_text = "ID:" + to_string(obj.id);
                            if (obj.number != -1)
                            {
                                display_text += " Num:" + to_string(obj.number);
                            }
                            putText(image, display_text,
                                    Point(rect.center.x + 20, rect.center.y - 30),
                                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 255, 255), 1);
                            break;
                        }
                    }
                    //

                    // // 显示透视变换结果
                    // if (!corrected_armor.empty())
                    // {
                    //     imshow("Corrected Armor", corrected_armor);
                    // }

                    if (approx.size() == 4 || approx.size() == 5 || approx.size() == 6)
                    {

                        //  在装甲板区域内直接检测红条
                        cv::Rect armor_roi = rect.boundingRect();
                        armor_roi &= cv::Rect(0, 0, image.cols, image.rows); // 确保在图像范围内

                        if (armor_roi.width > 0 && armor_roi.height > 0)
                        {
                            cv::Mat armor_region = image(armor_roi);

                            // 在装甲板区域内检测红色
                            cv::Mat hsv_armor;
                            cv::cvtColor(armor_region, hsv_armor, cv::COLOR_BGR2HSV);

                            cv::Mat mask1_armor, mask2_armor, mask_red_armor;
                            cv::inRange(hsv_armor, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1_armor);
                            cv::inRange(hsv_armor, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2_armor);
                            mask_red_armor = mask1_armor | mask2_armor;

                            vector<vector<Point>> contours_red_armor;
                            findContours(mask_red_armor, contours_red_armor, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                            std::vector<cv::Point2f> armor_red_points;

                            for (size_t k = 0; k < contours_red_armor.size(); k++)
                            {
                                double area_red = cv::contourArea(contours_red_armor[k]);
                                if ((area_red > 20) && (area_red < 500))
                                {
                                    // 计算最小外接矩形
                                    RotatedRect red_rect = minAreaRect(contours_red_armor[k]);

                                    // 将坐标转换回原图像坐标系
                                    Point2f red_center = red_rect.center;
                                    red_center.x += armor_roi.x;
                                    red_center.y += armor_roi.y;

                                    // 获取矩形的4个角点
                                    Point2f red_points[4];
                                    red_rect.points(red_points);

                                    // 将角点转换为vector并使用sortCorners排序
                                    std::vector<cv::Point2f> red_corners_vector;
                                    for (int j = 0; j < 4; j++)
                                    {
                                        red_points[j].x += armor_roi.x; // 坐标转换
                                        red_points[j].y += armor_roi.y;
                                        red_corners_vector.push_back(red_points[j]);
                                    }
                                    std::vector<cv::Point2f> sorted_red_corners = sortCorners(red_corners_vector);

                                    // 计算上下中点
                                    Point2f top_midpoint(
                                        (sorted_red_corners[1].x + sorted_red_corners[2].x) / 2,
                                        (sorted_red_corners[1].y + sorted_red_corners[2].y) / 2);
                                    Point2f bottom_midpoint(
                                        (sorted_red_corners[0].x + sorted_red_corners[3].x) / 2,
                                        (sorted_red_corners[0].y + sorted_red_corners[3].y) / 2);

                                    // 存储到装甲板的红条点列表
                                    armor_red_points.push_back(top_midpoint);
                                    armor_red_points.push_back(bottom_midpoint);

                                    // 在图像上标记（使用不同颜色区分）
                                    circle(image, top_midpoint, 4, Scalar(0, 255, 255), -1);    // 黄色：上中点
                                    circle(image, bottom_midpoint, 4, Scalar(255, 255, 0), -1); // 青色：下中点
                                    circle(image, red_center, 3, Scalar(0, 0, 255), -1);        // 红色：红条中心
                                }
                            }

                            // 存储装甲板的红条中点
                            if (armor_red_points.size() >= 4)
                            {
                                // 有足够多的红条点，取前4个
                                armor_red_points.resize(4);
                            }
                            else
                            {
                                // 红条点不足，用零填充
                                while (armor_red_points.size() < 4)
                                {
                                    armor_red_points.push_back(Point2f(0, 0));
                                }
                            }

                            // 存储到装甲板角点列表
                            armor_corners_list.push_back(armor_red_points);

                            RCLCPP_DEBUG(this->get_logger(), "装甲板%d找到%d个红条中点", current_id, (int)armor_red_points.size());
                        }
                    }
                }
            }
        }

        Mat image_rect = cv_ptr->image;
        // Mat image2 =cv_ptr->image;

        Mat hsv_rect, mask_rect;
        cvtColor(image_rect, hsv_rect, cv::COLOR_BGR2HSV);
        inRange(hsv_rect, Scalar(70, 80, 80), Scalar(100, 255, 255), mask_rect);
        vector<vector<Point>> contours_rect;
        findContours(mask_rect, contours_rect, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        vector<vector<Point2f>> conpoly_rect(contours_rect.size());
        for (size_t i = 0; i < contours_rect.size(); i++)
        {
            double area_rect = cv::contourArea(contours_rect[i]);
            if (area_rect > 50)
            {
                double peri_rect = arcLength(contours_rect[i], true);
                approxPolyDP(contours_rect[i], conpoly_rect[i], 0.02 * peri_rect, true);
                if (conpoly_rect[i].size() == 4)
                {

                    conpoly_rect[i] = sortCorners(conpoly_rect[i]);

                    vector<cv::Point2f> rect_corners;
                    for (int j = 0; j < 4; j++)
                    {
                        rect_corners.push_back(conpoly_rect[i][j]);
                    }
                    rectangle_corners_list.push_back(rect_corners);
                    rectangle_types.push_back("rectangle");

                    vector<string> corner_names = {"1", "2", "3", "4"}; // 左下、左上、右上、右下
                    vector<Scalar> colors = {
                        Scalar(0, 0, 255),  // 红色 - 左下
                        Scalar(0, 255, 0),  // 绿色 - 左上
                        Scalar(255, 0, 0),  // 蓝色 - 右上
                        Scalar(0, 255, 255) // 黄色 - 右下
                    };

                    for (int j = 0; j < 4; j++)
                    {
                        Point pt = Point(conpoly_rect[i][j].x, conpoly_rect[i][j].y);

                        // 画圆标记角点
                        circle(image_rect, pt, 8, colors[j], -1);

                        // 画外圈
                        circle(image_rect, pt, 10, Scalar(255, 255, 255), 1);

                        // 添加文字标签
                        putText(image_rect, corner_names[j], Point(pt.x + 15, pt.y - 10),
                                FONT_HERSHEY_SIMPLEX, 0.6, colors[j], 1);
                    }
                }
            }
        }

        cv::Mat result_image = image.clone();

        // 转换到 HSV 空间
        cv::Mat hsv_cir;
        cv::cvtColor(result_image, hsv_cir, cv::COLOR_BGR2HSV);

        // 红色检测 - 使用稳定的范围
        cv::Mat mask1, mask2, mask_cir;
        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255),
                    mask2);
        mask_cir = mask1 | mask2;

        // 适度的形态学操作
        cv::Mat kernel =
            cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask_cir, mask_cir, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask_cir, mask_cir, cv::MORPH_OPEN, kernel);

        // 找轮廓
        std::vector<std::vector<cv::Point>> contours_cir;
        cv::findContours(mask_cir, contours_cir, cv::RETR_EXTERNAL,
                         cv::CHAIN_APPROX_SIMPLE);

        int valid_spheres = 0;

        for (size_t i = 0; i < contours_cir.size(); i++)
        {
            double area = cv::contourArea(contours_cir[i]);
            if (area < 500)
                continue;

            // 计算最小外接圆
            Point2f center_cir;
            float radius = 0;
            minEnclosingCircle(contours_cir[i], center_cir, radius);

            // 计算圆形度
            double perimeter = arcLength(contours_cir[i], true);
            double circularity = 4 * CV_PI * area / (perimeter * perimeter);

            if (circularity > 0.7 && radius > 15 && radius < 200)
            {
                vector<Point2f> sphere_points =
                    calculateStableSpherePoints(center_cir, radius);

                sphere_corners_list.push_back(sphere_points);
                sphere_types.push_back("sphere");

                // 绘制检测到的球体
                cv::circle(image, center_cir, static_cast<int>(radius),
                           cv::Scalar(0, 255, 0), 2); // 绿色圆圈
                cv::circle(image, center_cir, 3, cv::Scalar(0, 0, 255),
                           -1); // 红色圆心

                // 绘制球体上的四个点
                vector<string> point_names = {"左", "下", "右", "上"};
                vector<cv::Scalar> point_colors = {
                    cv::Scalar(255, 0, 0),   // 蓝色 - 左
                    cv::Scalar(0, 255, 0),   // 绿色 - 下
                    cv::Scalar(0, 255, 255), // 黄色 - 右
                    cv::Scalar(255, 0, 255)  // 紫色 - 上
                };

                for (int j = 0; j < 4; j++)
                {
                    cv::circle(image, sphere_points[j], 6, point_colors[j], -1);
                    cv::circle(image, sphere_points[j], 6, cv::Scalar(0, 0, 0), 2);

                    // 标注序号
                    string point_text = to_string(j + 1);
                    cv::putText(
                        image, point_text,
                        cv::Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 3);
                    cv::putText(
                        image, point_text,
                        cv::Point(sphere_points[j].x + 10, sphere_points[j].y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, point_colors[j], 2);
                }
            }
        }

        //  发布消息部分
        referee_pkg::msg::MultiObject msg_object;
        msg_object.header = msg->header;

        // 计算总目标数
        int total_targets = armor_types.size() + rectangle_corners_list.size() + sphere_corners_list.size();
        msg_object.num_objects = total_targets;

        // 1. 发布装甲板（类型，角点）
        for (int i = 0; i < armor_types.size(); i++)
        {
            referee_pkg::msg::Object obj;
            obj.target_type = armor_types[i];

            // 使用红条中点作为角点
            for (int j = 0; j < 4; j++)
            {
                geometry_msgs::msg::Point corner;
                if (i < armor_corners_list.size() && j < armor_corners_list[i].size())
                {
                    corner.x = armor_corners_list[i][j].x;
                    corner.y = armor_corners_list[i][j].y;
                }
                else
                {
                    corner.x = 0.0; // 安全保护
                    corner.y = 0.0;
                }
                corner.z = 0.0;
                obj.corners.push_back(corner);
            }

            msg_object.objects.push_back(obj);

            RCLCPP_DEBUG(this->get_logger(), "装甲板%d: 类型=%s, 角点数=%d",
                         i, armor_types[i].c_str(), (int)armor_corners_list[i].size());
        }

        // 2. 发布矩形（类型+4个角点）
        for (int i = 0; i < rectangle_corners_list.size(); i++)
        {
            referee_pkg::msg::Object obj;
            obj.target_type = "rectangle";

            for (int j = 0; j < 4; j++)
            {
                geometry_msgs::msg::Point corner;
                corner.x = rectangle_corners_list[i][j].x;
                corner.y = rectangle_corners_list[i][j].y;
                corner.z = 0.0;
                obj.corners.push_back(corner);
            }

            msg_object.objects.push_back(obj);
        }

        // 3. 发布球体（类型+4个角点）
        for (int i = 0; i < sphere_corners_list.size(); i++)
        {
            referee_pkg::msg::Object obj;
            obj.target_type = "sphere";

            for (int j = 0; j < 4; j++)
            {
                geometry_msgs::msg::Point corner;
                corner.x = sphere_corners_list[i][j].x;
                corner.y = sphere_corners_list[i][j].y;
                corner.z = 0.0;
                obj.corners.push_back(corner);
            }

            msg_object.objects.push_back(obj);
        }

        Target_pub->publish(msg_object);

        RCLCPP_INFO(this->get_logger(), "发布完成: %d个装甲板(仅类型) %d个矩形 %d个球体, 总计%d个目标",
                    (int)armor_types.size(), (int)rectangle_corners_list.size(), (int)sphere_corners_list.size(), total_targets);
        cv::imshow("Detection Result", image);
        cv::waitKey(1);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocateNode>("LocateNode");
    RCLCPP_INFO(node->get_logger(), "Starting TestNode");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
