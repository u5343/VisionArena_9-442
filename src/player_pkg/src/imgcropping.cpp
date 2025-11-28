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

#include "sensor_msgs/msg/image.hpp"

using namespace std;
using namespace rclcpp;
using namespace cv;

class WarmNode : public rclcpp::Node
{
public:
    WarmNode(string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Initializing WarmNode with corner detection");
        Image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10,
            bind(&WarmNode::callback_camera, this, std::placeholders::_1));
        Target_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "/edited_camera/image_raw", 10);
        cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Corrected Armor", cv::WINDOW_AUTOSIZE);
        RCLCPP_INFO(this->get_logger(), "WarmNode initialized successfully");
    }
    ~WarmNode()
    {
        cv::destroyWindow("Detection Result");
        cv::destroyWindow("Corrected Armor");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr Target_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    Mat imgcropping(const Mat &srcimg)
    {
        // 1. 转换为灰度图
        Mat gray;
        cvtColor(srcimg, gray, COLOR_BGR2GRAY);

        // 2. 二值化
        Mat binary;
        threshold(gray, binary, 127, 255, THRESH_BINARY);

        // 【关键】反转二值化图像，让黑色矩形变成白色
        Mat binary_inv;
        bitwise_not(binary, binary_inv);

        // 调试：显示反转后的图像
        cv::imshow("Binary Inverted", binary_inv);

        // 3. 查找轮廓（现在找的是原来的黑色矩形）
        vector<vector<Point>> contours;
        findContours(binary_inv.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contours.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No contours found!");
            return srcimg.clone();
        }

        // 4. 找到最大轮廓（应该是黑色矩形框）
        int idx = 0;
        double maxArea = 0;
        for (int i = 0; i < contours.size(); i++)
        {
            double area = contourArea(contours[i]);
            if (area > maxArea)
            {
                maxArea = area;
                idx = i;
            }
        }

        RCLCPP_INFO(this->get_logger(), "Detected rectangle area: %.2f", maxArea);

        // 5. 获取最小外接旋转矩形
        RotatedRect rotatedRect = minAreaRect(contours[idx]);

        // 6. 获取四个角点
        Point2f vertices[4];
        rotatedRect.points(vertices);
        vector<Point2f> srcpoints(vertices, vertices + 4);

        // 7. 对角点排序：找出左上、右上、右下、左下
        // 先按y坐标排序
        sort(srcpoints.begin(), srcpoints.end(), [](Point2f a, Point2f b)
             { return a.y < b.y; });

        // 上面两个点按x排序
        Point2f tl = srcpoints[0].x < srcpoints[1].x ? srcpoints[0] : srcpoints[1];
        Point2f tr = srcpoints[0].x < srcpoints[1].x ? srcpoints[1] : srcpoints[0];

        // 下面两个点按x排序
        Point2f bl = srcpoints[2].x < srcpoints[3].x ? srcpoints[2] : srcpoints[3];
        Point2f br = srcpoints[2].x < srcpoints[3].x ? srcpoints[3] : srcpoints[2];

        vector<Point2f> orderedSrc = {tl, tr, br, bl};

        // 8. 计算目标尺寸（保持宽高比）
        float width = norm(tr - tl);
        float height = norm(bl - tl);

        // 或者使用固定尺寸
        // float width = 100.0f;
        // float height = 50.0f;

        vector<Point2f> dstpoints = {
            Point2f(0, 0),          // 左上
            Point2f(width, 0),      // 右上
            Point2f(width, height), // 右下
            Point2f(0, height)      // 左下
        };

        // 9. 绘制调试信息
        Mat debug = srcimg.clone();
        drawContours(debug, contours, idx, Scalar(0, 255, 0), 2);

        vector<string> labels = {"TL", "TR", "BR", "BL"};
        for (int i = 0; i < orderedSrc.size(); i++)
        {
            circle(debug, orderedSrc[i], 5, Scalar(0, 0, 255), -1);
            putText(debug, labels[i], orderedSrc[i] + Point2f(10, 10),
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 2);
        }
        cv::imshow("Debug Points", debug);

        // 10. 透视变换
        Mat transform_matrix = getPerspectiveTransform(orderedSrc, dstpoints);
        Mat corrected;
        warpPerspective(srcimg, corrected, transform_matrix, Size(width, height));

        return corrected;
    }

    void callback_camera(sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat image = cv_ptr->image;
        Mat resImg;
        resImg = imgcropping(image);
        cv::imshow("Detection Result", resImg);
        cv::waitKey(1);
        auto result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", resImg).toImageMsg();
        Target_pub->publish(*result_msg); // 解引用智能指针
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WarmNode>("RecognizeNode");
    RCLCPP_INFO(node->get_logger(), "Starting RecognizeNode.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}