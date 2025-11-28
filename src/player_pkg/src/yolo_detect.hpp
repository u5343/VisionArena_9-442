#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <algorithm>

struct Detection
{
  cv::Rect box;
  float confidence;
  int class_id;
};

class YOLODetector : public rclcpp::Node
{
public:
  YOLODetector() : Node("yolo_detector")
  {
    // 初始化ONNXRuntime
    initONNX();

    // 订阅图像话题
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera/image_raw", 10,
        std::bind(&YOLODetector::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "YOLO Detector initialized");
    std::string getLatestArmor() const
    {
      return latest_armor_;
    }
  }

private:
  void initONNX()
  {
    try
    {
      // 创建ONNX环境
      env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLOv11");

      // 会话选项
      Ort::SessionOptions session_options;
      session_options.SetIntraOpNumThreads(4);
      session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

      // 如果有GPU可用，启用CUDA
      // OrtCUDAProviderOptions cuda_options;
      // session_options.AppendExecutionProvider_CUDA(cuda_options);

      // 加载模型
      const char *model_path = "/home/jiangping/YOLO/models/best.onnx";
      session_ = std::make_unique<Ort::Session>(*env_, model_path, session_options);

      // 获取输入输出信息
      Ort::AllocatorWithDefaultOptions allocator;

      // 输入信息
      input_name_ = session_->GetInputNameAllocated(0, allocator).get();
      auto input_shape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      input_height_ = input_shape[2];
      input_width_ = input_shape[3];

      // 输出信息
      output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

      RCLCPP_INFO(this->get_logger(), "Model loaded: input size %ldx%ld",
                  input_height_, input_width_);
    }
    catch (const Ort::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "ONNX Error: %s", e.what());
      throw;
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try
    {
      // 转换ROS图像为OpenCV格式
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      cv::Mat image = cv_ptr->image;

      // 预处理
      cv::Mat blob = preprocess(image);

      // 推理
      auto detections = inference(blob, image.size());

      // 可视化结果
      visualize(image, detections);
      if (!detections.empty())
      {
        int id = detections[0].class_id + 1;
        latest_armor_ = "armor" + std::to_string(id);
      }
      else
      {
        latest_armor_ = "none";
      }
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  cv::Mat preprocess(const cv::Mat &image)
  {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(input_width_, input_height_));

    // 转换为float并归一化到[0,1]
    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    // 转换为NCHW格式
    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0, cv::Size(input_width_, input_height_),
                           cv::Scalar(0, 0, 0), true, false);

    return blob;
  }

  std::vector<Detection> inference(const cv::Mat &blob, const cv::Size &original_size)
  {
    // 准备输入张量
    std::vector<int64_t> input_shape = {1, 3, input_height_, input_width_};
    size_t input_tensor_size = blob.total() * blob.elemSize() / sizeof(float);

    auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info, (float *)blob.data, input_tensor_size,
        input_shape.data(), input_shape.size());

    // 运行推理
    const char *input_names[] = {input_name_.c_str()};
    const char *output_names[] = {output_name_.c_str()};

    auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                        input_names, &input_tensor, 1,
                                        output_names, 1);

    // 获取输出
    float *output_data = output_tensors[0].GetTensorMutableData<float>();
    auto output_shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    // 后处理
    return postprocess(output_data, output_shape, original_size);
  }

  std::vector<Detection> postprocess(float *output, const std::vector<int64_t> &shape,
                                     const cv::Size &original_size)
  {
    std::vector<Detection> detections;

    // YOLOv11输出格式: [1, 84, 8400] (80类+4个box坐标)
    // 或者 [1, num_classes+4, num_boxes]
    int num_boxes = shape[2];
    int num_data = shape[1];
    int num_classes = num_data - 4;

    float conf_threshold = 0.25f;
    float nms_threshold = 0.45f;

    // 缩放因子
    float x_scale = original_size.width / (float)input_width_;
    float y_scale = original_size.height / (float)input_height_;

    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> class_ids;

    // 遍历所有检测框
    for (int i = 0; i < num_boxes; ++i)
    {
      // 获取类别分数
      float max_score = 0.0f;
      int max_class_id = 0;

      for (int j = 4; j < num_data; ++j)
      {
        float score = output[j * num_boxes + i];
        if (score > max_score)
        {
          max_score = score;
          max_class_id = j - 4;
        }
      }

      if (max_score > conf_threshold)
      {
        // 获取边界框坐标 (中心点格式)
        float cx = output[0 * num_boxes + i] * x_scale;
        float cy = output[1 * num_boxes + i] * y_scale;
        float w = output[2 * num_boxes + i] * x_scale;
        float h = output[3 * num_boxes + i] * y_scale;

        // 转换为左上角坐标
        int x = static_cast<int>(cx - w / 2);
        int y = static_cast<int>(cy - h / 2);

        boxes.push_back(cv::Rect(x, y, static_cast<int>(w), static_cast<int>(h)));
        confidences.push_back(max_score);
        class_ids.push_back(max_class_id);
      }
    }

    // 非极大值抑制
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);

    for (int idx : indices)
    {
      Detection det;
      det.box = boxes[idx];
      det.confidence = confidences[idx];
      det.class_id = class_ids[idx];
      detections.push_back(det);
    }

    return detections;
  }

  void visualize(cv::Mat &image, const std::vector<Detection> &detections)
  {
    for (const auto &det : detections)
    {
      // 绘制边界框
      cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);

      // 绘制标签
      std::string label = "Armor" + std::to_string(det.class_id + 1) +
                          ": " + std::to_string(int(det.confidence * 100)) + "%";

      int baseline;
      cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
      if (det.confidence * 100 > 80)
      {
        cv::rectangle(image,
                      cv::Point(det.box.x, det.box.y - text_size.height - 5),
                      cv::Point(det.box.x + text_size.width, det.box.y),
                      cv::Scalar(0, 255, 0), -1);

        cv::putText(image, label,
                    cv::Point(det.box.x, det.box.y - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
      }
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_;
  std::string output_name_;
  int64_t input_height_;
  int64_t input_width_;
  std::string latest_armor = "none";
};
