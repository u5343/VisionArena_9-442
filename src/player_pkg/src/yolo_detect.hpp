#pragma once
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>
#include <vector>
#include <memory>

struct Detection
{
  cv::Rect box;
  float confidence;
  int class_id;
};

class YOLODetector
{
public:
  YOLODetector(const std::string &model_path = "/home/jiangping/YOLO/models/best.onnx")
  {
    initONNX(model_path);
  }

  // ---------------------------------------
  //   核心接口：对图像执行 YOLO 推理
  // ---------------------------------------
  int detect(const cv::Mat &image)
  {
    if (image.empty())
      return -1;

    cv::Mat blob = preprocess(image);
    auto dets = inference(blob, image.size());

    if (!dets.empty())
    {
      int id = dets[0].class_id + 1;
      return id;
    }
    return -1;
  }

private:
  // ---------------------------------------
  //   初始化 ONNX
  // ---------------------------------------
  void initONNX(const std::string &model_path)
  {
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "YOLOv11");

    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(4);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), opts);

    Ort::AllocatorWithDefaultOptions allocator;

    input_name_ = session_->GetInputNameAllocated(0, allocator).get();
    output_name_ = session_->GetOutputNameAllocated(0, allocator).get();

    auto ishape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    input_h_ = ishape[2];
    input_w_ = ishape[3];
  }

  // ---------------------------------------
  //   预处理
  // ---------------------------------------
  cv::Mat preprocess(const cv::Mat &img)
  {
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(input_w_, input_h_));

    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    cv::Mat blob;
    cv::dnn::blobFromImage(resized, blob, 1.0, cv::Size(input_w_, input_h_),
                           cv::Scalar(), true, false);
    return blob;
  }

  // ---------------------------------------
  //   推理
  // ---------------------------------------
  std::vector<Detection> inference(const cv::Mat &blob, cv::Size osize)
  {
    std::vector<int64_t> input_dims = {1, 3, input_h_, input_w_};

    auto mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem, (float *)blob.data, blob.total(), input_dims.data(), input_dims.size());

    const char *inputs[] = {input_name_.c_str()};
    const char *outputs[] = {output_name_.c_str()};

    auto output_tensors = session_->Run(Ort::RunOptions{nullptr},
                                        inputs, &input_tensor, 1,
                                        outputs, 1);

    float *output = output_tensors[0].GetTensorMutableData<float>();
    auto shape = output_tensors[0].GetTensorTypeAndShapeInfo().GetShape();

    return postprocess(output, shape, osize);
  }

  // ---------------------------------------
  //   后处理
  // ---------------------------------------
  std::vector<Detection> postprocess(float *out, const std::vector<int64_t> &shape, cv::Size osize)
  {
    std::vector<Detection> dets;

    int num_boxes = shape[2];
    int feat_len = shape[1];
    int num_classes = feat_len - 4;

    float sx = osize.width / float(input_w_);
    float sy = osize.height / float(input_h_);

    for (int i = 0; i < num_boxes; i++)
    {
      float best = 0;
      int cid = -1;
      for (int j = 0; j < num_classes; j++)
      {
        float score = out[(4 + j) * num_boxes + i];
        if (score > best)
        {
          best = score;
          cid = j;
        }
      }

      if (best < 0.25)
        continue;

      float cx = out[0 * num_boxes + i] * sx;
      float cy = out[1 * num_boxes + i] * sy;
      float w = out[2 * num_boxes + i] * sx;
      float h = out[3 * num_boxes + i] * sy;

      Detection d;
      d.class_id = cid;
      d.confidence = best;
      d.box = cv::Rect(cv::Point(cx - w / 2, cy - h / 2), cv::Size(w, h));

      dets.push_back(d);
    }

    return dets;
  }

private:
  std::unique_ptr<Ort::Env> env_;
  std::unique_ptr<Ort::Session> session_;
  std::string input_name_, output_name_;
  int64_t input_h_, input_w_;
};
