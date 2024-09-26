#include "srm/autoaim/autoaim-armor.h"

#include "srm/autoaim/drawer.h"

namespace srm::autoaim {

bool ArmorAutoaim::Initialize() {
  /// 公共初始化
  bool ret = BaseAutoaim::Initialize();
  /// 初始化detector,chooser,processor
  armor_detector_ = std::make_unique<ArmorDetector>();
  ret &= armor_detector_->Initialize();

  if (!ret) {
    LOG(ERROR) << "Failed to initialize autoaim for armor.";
    return false;
  }
  return true;
}

bool ArmorAutoaim::Run() {
  ArmorPtrList armor_list_;
  /// 完成识别和处理，最终需要得到yaw_和pitch_的数据，注意这两个数据并不是相对角，而是要根据电控传来的rm_self_来进行计算其绝对角
  /// 如果未识别到，请发送0，这个时候机器人会自动进行视野的扫描，但如果想要自行在没识别到的时候也要自己操纵机器人的方向，也可不赋值为0

  // 运行detector，获得识别信息
  armor_detector_->Run(image_, armor_list_);

  if (armor_list_.empty()) {
    // 如果未识别到
    yaw_ = 0;
    pitch_ = 0;
    return false;
  }

  // 获取第一个数据（替换为置信度最高的？）
  auto armor = armor_list_.front();

  // 计算中心点
  cv::Point2f center = (armor->pts[0] + armor->pts[1] + armor->pts[2] + armor->pts[3]) * 0.25;

  // 使用 Solver 类来转换坐标
  coord::Solver solver_trans;
  // 假设 solver 已经初始化并且相关参数已经设置
  coord::CTVec cam_coords(center.x, center.y,0);

  // 获得一个在时空中绝对的坐标系
  coord::RMat rm_imu = rm_self_;
  coord::CTVec world_coords = solver_trans.CamToWorld(cam_coords, rm_imu);

  /// 计算在 xy 平面上的距离
  float xy_distance = sqrt(world_coords(0) * world_coords(0) + world_coords(1) * world_coords(1));

  // 计算偏航角和俯仰角
  yaw_ = atan2(static_cast<float>(world_coords(1)), static_cast<float>(world_coords(0)));
  pitch_ = atan2(static_cast<float>(world_coords(2)), xy_distance);

#ifdef DEBUG
  viewer_->SendFrame(image_);
#endif

  return true;
}
bool ArmorAutoaim::InitializeViewerImpl() {
  return viewer_->Initialize(cfg.Get<std::string>({"viewer.web.shm_name", "armor"}),
                             cfg.Get<int>({"viewer.web", "armor"}));
}

}  // namespace srm::autoaim
