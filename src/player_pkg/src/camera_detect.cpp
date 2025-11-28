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
#include <std_msgs/msg/header.hpp>
#include "yolo_detect.hpp"

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
        detector = std::make_shared<YOLODetector>();
        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "TestNode initialized successfully");
    }
    ~LocateNode() { cv::destroyWindow("Detection Result"); }

private:
    std::shared_ptr<YOLODetector> detector;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<referee_pkg::msg::MultiObject>::SharedPtr Target_pub;

    std::vector<std::vector<cv::Point2f>> rectangle_corners_list; // 矩形角点列表
    std::vector<std::vector<cv::Point2f>> sphere_corners_list;    // 球体角点列表
    std::vector<std::vector<cv::Point2f>> midpoint_corners_list;

    // 对应的目标类型
    std::vector<std::string> armor_types;     // 装甲板类型
    std::vector<std::string> rectangle_types; // 矩形类型
    std::vector<std::string> sphere_types;    // 球体类型

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

    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg)
    {

        armor_types.clear();
        rectangle_corners_list.clear();
        rectangle_types.clear();
        sphere_corners_list.clear();
        sphere_types.clear();
        midpoint_corners_list.clear();

        // 图像转换
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat image = cv_ptr->image;
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
                    // 使用最小外接矩形找4个有序角点
                    RotatedRect rect = minAreaRect(approx); // 最小外接矩形

                    Point2f rect_points[4];
                    rect.points(rect_points); // 直接得到4个有序角点

                    // 绘制矩形角点（用绿色标记）
                    for (int j = 0; j < 4; j++)
                    {
                        circle(image, rect_points[j], 8, Scalar(0, 255, 0), 1);
                    }
                    std::string armor = detector->getLatestArmor();
                    RCLCPP_INFO(this->get_logger(), "detected:%s", armor.c_str());
                    // 简化：直接添加装甲板类型
                    armor_types.push_back(armor);
                }
            }
        }

        Mat image_rect = cv_ptr->image;

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
                    vector<cv::Point2f> rect_corners;
                    for (int j = 0; j < 4; j++)
                    {
                        rect_corners.push_back(conpoly_rect[i][j]);
                    }
                    rectangle_corners_list.push_back(rect_corners);
                    rectangle_types.push_back("rectangle");

                    vector<string> corner_names = {"1", "2", "3", "4"};
                    vector<Scalar> colors = {
                        Scalar(0, 0, 255),  // 红色
                        Scalar(0, 255, 0),  // 绿色
                        Scalar(255, 0, 0),  // 蓝色
                        Scalar(0, 255, 255) // 黄色
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

        Mat image_red = cv_ptr->image;
        cv::Mat hsv_red;
        cv::cvtColor(image_red, hsv_red, cv::COLOR_BGR2HSV);

        // 红色检测 - 使用稳定的范围
        cv::Mat mask1_red, mask2_red, mask_red;
        cv::inRange(hsv_red, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1_red);
        cv::inRange(hsv_red, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2_red);
        mask_red = mask1_red | mask2_red;

        vector<vector<Point>> contours_red;
        findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        for (size_t i = 0; i < contours_red.size(); i++)
        {
            double area_red = cv::contourArea(contours_red[i]);
            if ((area_red > 20) && (area_red < 500))
            {
                // 计算最小外接矩形
                RotatedRect min_rect = minAreaRect(contours_red[i]);

                // 获取矩形的4个角点
                Point2f rect_points[4];
                min_rect.points(rect_points);

                // 简化：直接计算中点
                // 计算所有点的平均位置来找上下边
                Point2f center = min_rect.center;

                // 找到上边和下边的点
                vector<Point2f> top_points, bottom_points;
                for (int j = 0; j < 4; j++)
                {
                    if (rect_points[j].y < center.y)
                        top_points.push_back(rect_points[j]);
                    else
                        bottom_points.push_back(rect_points[j]);
                }

                // 计算上边和下边的中点
                Point2f top_midpoint(0, 0), bottom_midpoint(0, 0);

                if (top_points.size() >= 2)
                {
                    top_midpoint = Point2f(
                        (top_points[0].x + top_points[1].x) / 2,
                        (top_points[0].y + top_points[1].y) / 2);
                }

                if (bottom_points.size() >= 2)
                {
                    bottom_midpoint = Point2f(
                        (bottom_points[0].x + bottom_points[1].x) / 2,
                        (bottom_points[0].y + bottom_points[1].y) / 2);
                }

                // 存储中点坐标（模拟4个点）
                std::vector<cv::Point2f> midpoint_corners;
                midpoint_corners.push_back(top_midpoint);    // 点1：上中点
                midpoint_corners.push_back(bottom_midpoint); // 点2：下中点
                midpoint_corners.push_back(top_midpoint);    // 点3：（占位）
                midpoint_corners.push_back(bottom_midpoint); // 点4：（占位）

                midpoint_corners_list.push_back(midpoint_corners);

                // 用圆圈标记这两个中点
                circle(image, top_midpoint, 4, Scalar(0, 255, 0), -1);    // 绿色：上中点
                circle(image, bottom_midpoint, 4, Scalar(0, 255, 0), -1); // 绿色：下中点
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

            // 为每个装甲板找到最近的两个红条中点作为角点
            if (i < midpoint_corners_list.size())
            {
                // 直接使用对应的红条中点坐标
                for (int j = 0; j < 4; j++)
                {
                    geometry_msgs::msg::Point corner;
                    corner.x = midpoint_corners_list[i][j].x;
                    corner.y = midpoint_corners_list[i][j].y;
                    corner.z = 0.0;
                    obj.corners.push_back(corner);
                }
            }
            else
            {

                for (int j = 0; j < 4; j++)
                {
                    geometry_msgs::msg::Point corner;
                    corner.x = 0.0;
                    corner.y = 0.0;
                    corner.z = 0.0;
                    obj.corners.push_back(corner);
                }
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