#include "srm/autoaim/detector-armor.h"

namespace srm::autoaim {

bool ArmorDetector::Initialize() {
  /// 初始化yolo
#if defined(__APPLE__)
  std::string net_type = "coreml";
#elif defined(__linux__)
  std::string net_type = "tensorrt";
#endif
  yolo_.reset(nn::CreateYolo(net_type));
  std::string prefix = "nn.yolo.armor";
  const auto model_path = cfg.Get<std::string>({prefix, net_type});
  const auto class_num = cfg.Get<int>({prefix, "class_num"});
  const auto point_num = cfg.Get<int>({prefix, "point_num"});
  if (!yolo_->Initialize(model_path, class_num, point_num)) {
    LOG(ERROR) << "Failed to load armor nerual network.";
    return false;
  }
  return true;
}

bool ArmorDetector::Run(cv::Mat REF_IN image, ArmorPtrList REF_OUT armor_list) const {
  ///请补全

  //检查输入图像是否为空
  if (image.empty()) {
    LOG(ERROR) << "Input image is empty."; // 记录错误日志
    return false; // 返回失败
  }

  //使用YOLO进行目标检测
  //运行神经网络检测
   std::vector<srm::nn::Objects> detections = yolo_->Run(image);

  for(const auto& obj : detections) {
    //置信度

    //计算装甲板的四个角点
    cv::Point2f top_left(obj.x1, obj.y1);
    cv::Point2f bottom_right(obj.x2, obj.y2);
    cv::Point2f bottom_left(top_left.x, bottom_right.y);
    cv::Point2f top_right(bottom_right.x, top_left.y);

    //创建一个color对象，传入颜色
    Color lamp_color = (obj.cls == 0) ? Color::kBlue : Color::kRed;

    //智能指针，用来储存灯泡信息
    ArmorPtr lamp = std::make_shared<Armor>(std::array<cv::Point2f, 4> {top_left,top_right,bottom_left,bottom_right}, lamp_color);

    //创建一个vector，用来返回信息
    armor_list.push_back(lamp);
  }

  return true;
}

}  // namespace srm::autoaim