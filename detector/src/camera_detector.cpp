#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using namespace cv;

class detector : public rclcpp::Node
{
public:
    detector() : Node("detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "Detector_node launched.");
    }
    void init()
    {
        RCLCPP_INFO(this->get_logger(), "Initializing...");
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        sub_ = it_->subscribe("/camera/image_raw", 1, std::bind(&detector::image_callback, this, std::placeholders::_1));
        pub_ = it_->advertise("detector/result", 1);
    }

private:
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), e.what());
            return;
        }
        Mat result_image = this->findObjects(cv_ptr->image);
        sensor_msgs::msg::Image::SharedPtr result_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result_image).toImageMsg();
        pub_.publish(result_msg);
    }

    Mat findObjects(const Mat &srcimg)
    {
        int hmin_r = 0;
        int hmax_r = 20;
        int smin_r = 100;
        int smax_r = 255;
        int vmin_r = 100;
        int vmax_r = 255;
        Scalar lower_r(hmin_r, smin_r, vmin_r);
        Scalar upper_r(hmax_r, smax_r, vmax_r);
        Mat hsvimg;
        cvtColor(srcimg, hsvimg, COLOR_BGR2HSV);
        Mat mask_r;
        inRange(hsvimg, lower_r, upper_r, mask_r);
        std::vector<std::vector<Point>> contours;
        findContours(mask_r, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        std::vector<Rect> boundRect(contours.size());
        Mat outputimg = srcimg.clone();
        for (int i = 0; i < contours.size(); i++)
        {
            boundRect[i] = boundingRect(contours[i]);
            rectangle(outputimg, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2);
        }
        return outputimg;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // 4. 修改 main 函数
    // 之前: rclcpp::spin(std::make_shared<BallDetector>());

    // 之后:
    // 步骤 A: 创建 shared_ptr
    auto node = std::make_shared<detector>();

    // 步骤 B: (现在 shared_ptr 已存在) 调用 init()
    node->init();

    // 步骤 C: 自旋
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}