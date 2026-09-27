// 合成传感器数据源（无实车验证用）
//
// 发布：
//   /points_raw            sensor_msgs/PointCloud2   10Hz 模拟激光（两个目标：运动 + 静止）
//   /perception/yolo_boxes lidar_msgs/VisionBoxes    10Hz 模拟 YOLO 框（与 camera.json 投影一致）
//
// 毫米波目标由 radar_adapter(sim) 独立发布到 /sensorRawData，与这里的静止/运动目标位置一致，
// 以便端到端跑通 感知融合（激光+雷达+相机）→ fusion_cells → decision_planning。
//
// 运动目标：x 从 moving_x0=30 以 -5m/s 靠近，横向 y=-2（右侧 2m）
// 静止目标：固定在 (15, 3)
//
// 坐标系约定与 perception 一致：车辆系，x 前 / y 左 / z 上。
// 点云消息手工构造（仅 x/y/z/intensity），与 perception 的 PointXYZIART 注册字段匹配。

#include <cstdint>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "lidar_msgs/msg/vision_boxes.hpp"

namespace sim_sensors
{

// 与 perception/config_json/camera.json 一致的粗标定（名义前置相机）
struct CameraCalib
{
  double fx = 1000.0, fy = 1000.0, cx = 640.0, cy = 360.0;
  double t_x = 1.2, t_y = 0.0, t_z = 1.5;
  int width = 1280, height = 720;

  // 车辆系 (xv,yv,zv) → 像素 (u,v)。Rv2c=R0（yaw=pitch=roll=0）
  bool project(double xv, double yv, double zv, double * u, double * v) const
  {
    const double px = xv - t_x, py = yv - t_y, pz = zv - t_z;
    const double Xc = -py;          // R0 行1
    const double Yc = -pz;          // R0 行2
    const double Zc = px;           // R0 行3
    if (Zc <= 0.01) return false;
    const double uu = fx * Xc / Zc + cx;
    const double vv = fy * Yc / Zc + cy;
    if (uu < 0 || uu >= width || vv < 0 || vv >= height) return false;
    *u = uu;
    *v = vv;
    return true;
  }
};

class SimSensors : public rclcpp::Node
{
public:
  SimSensors() : Node("sim_sensors")
  {
    declare_parameter("rate_hz", 10.0);
    declare_parameter("lidar_topic", "/points_raw");
    declare_parameter("yolo_topic", "/perception/yolo_boxes");
    declare_parameter("moving_x0", 30.0);
    declare_parameter("moving_vx", -5.0);
    declare_parameter("moving_y", -2.0);
    declare_parameter("static_x", 15.0);
    declare_parameter("static_y", 3.0);

    rate_hz_ = get_parameter("rate_hz").as_double();
    moving_x_ = get_parameter("moving_x0").as_double();
    moving_vx_ = get_parameter("moving_vx").as_double();
    moving_y_ = get_parameter("moving_y").as_double();
    static_x_ = get_parameter("static_x").as_double();
    static_y_ = get_parameter("static_y").as_double();

    const std::string lidar_topic = get_parameter("lidar_topic").as_string();
    const std::string yolo_topic = get_parameter("yolo_topic").as_string();
    pub_lidar_ = create_publisher<sensor_msgs::msg::PointCloud2>(lidar_topic, 10);
    pub_yolo_ = create_publisher<lidar_msgs::msg::VisionBoxes>(yolo_topic, 10);

    const int period_ms = static_cast<int>(1000.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SimSensors::tick, this));

    RCLCPP_INFO(get_logger(),
                "sim_sensors @ %.1fHz: lidar=%s yolo=%s moving=(%.1f,%.1f)@%.1f static=(%.1f,%.1f)",
                rate_hz_, lidar_topic.c_str(), yolo_topic.c_str(),
                moving_x_, moving_y_, moving_vx_, static_x_, static_y_);
  }

private:
  // 一个目标的点云幕墙：密集点跨越 z 0.3~1.5m，保证 radius 滤波 + 地面分割后保留非地面点
  void addCurtain(std::vector<float> & xyz, double cx, double cy) const
  {
    for (double z = 0.3; z <= 1.5; z += 0.08) {
      for (double dx = -0.6; dx <= 0.6; dx += 0.06) {
        for (double dy = -0.6; dy <= 0.6; dy += 0.06) {
          xyz.push_back(static_cast<float>(cx + dx));
          xyz.push_back(static_cast<float>(cy + dy));
          xyz.push_back(static_cast<float>(z));
        }
      }
    }
  }

  // 手工构造 PointCloud2：fields = [x,y,z,intensity]，全部 FLOAT32，按点交错存储
  sensor_msgs::msg::PointCloud2 makeCloud(const std::vector<float> & xyz) const
  {
    const size_t n = xyz.size() / 3;
    sensor_msgs::msg::PointCloud2 out;
    out.height = 1;
    out.width = static_cast<uint32_t>(n);
    out.is_bigendian = false;
    out.is_dense = true;
    out.point_step = 16;   // 4 * 4B
    out.row_step = out.point_step * out.width;

    out.fields.resize(4);
    out.fields[0].name = "x";
    out.fields[0].offset = 0;
    out.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[0].count = 1;
    out.fields[1].name = "y";
    out.fields[1].offset = 4;
    out.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[1].count = 1;
    out.fields[2].name = "z";
    out.fields[2].offset = 8;
    out.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[2].count = 1;
    out.fields[3].name = "intensity";
    out.fields[3].offset = 12;
    out.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    out.fields[3].count = 1;

    // 交错布局：每点 [x,y,z,intensity] 共 16B
    std::vector<float> buf(n * 4);
    for (size_t i = 0; i < n; ++i) {
      buf[i * 4 + 0] = xyz[i * 3 + 0];
      buf[i * 4 + 1] = xyz[i * 3 + 1];
      buf[i * 4 + 2] = xyz[i * 3 + 2];
      buf[i * 4 + 3] = 50.0f;   // intensity
    }
    out.data.resize(buf.size() * 4);
    std::memcpy(out.data.data(), buf.data(), buf.size() * 4);
    return out;
  }

  void appendBox(lidar_msgs::msg::VisionBoxes & boxes, double xv, double yv, int cls_id,
                 float score) const
  {
    double u, v;
    if (!calib_.project(xv, yv, 1.6, &u, &v)) return;

    // 框宽高随深度缩放（近似 2.5m 宽 × 2.0m 高的目标）
    const double depth = std::max(1.0, xv - calib_.t_x);
    const double w = 2.5 * calib_.fx / depth;
    const double h = 2.0 * calib_.fy / depth;

    lidar_msgs::msg::VisionBox b;
    b.x1 = static_cast<float>(std::max(0.0, u - w / 2.0));
    b.y1 = static_cast<float>(std::max(0.0, v - h / 2.0));
    b.x2 = static_cast<float>(std::min(static_cast<double>(calib_.width), u + w / 2.0));
    b.y2 = static_cast<float>(std::min(static_cast<double>(calib_.height), v + h / 2.0));
    b.cls_id = cls_id;
    b.score = score;
    boxes.boxes.push_back(b);
  }

  void tick()
  {
    moving_x_ += moving_vx_ / rate_hz_;
    if (moving_x_ < 4.0) moving_x_ = 4.0;   // 不穿到自车

    const auto now = this->now();

    // ---- 1. 模拟激光 ----
    std::vector<float> xyz;
    addCurtain(xyz, moving_x_, moving_y_);
    addCurtain(xyz, static_x_, static_y_);
    sensor_msgs::msg::PointCloud2 out = makeCloud(xyz);
    out.header.frame_id = "world";
    out.header.stamp = now;
    pub_lidar_->publish(out);

    // ---- 2. 模拟 YOLO 检测框 ----
    lidar_msgs::msg::VisionBoxes boxes;
    boxes.header.stamp = now;
    boxes.image_width = calib_.width;
    boxes.image_height = calib_.height;
    appendBox(boxes, moving_x_, moving_y_, 2, 0.92f);   // 运动目标 → car(cls2)
    appendBox(boxes, static_x_, static_y_, 0, 0.85f);   // 静止目标 → person(cls0)
    pub_yolo_->publish(boxes);
  }

  CameraCalib calib_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_lidar_;
  rclcpp::Publisher<lidar_msgs::msg::VisionBoxes>::SharedPtr pub_yolo_;
  rclcpp::TimerBase::SharedPtr timer_;
  double rate_hz_{10.0};
  double moving_x_{30.0}, moving_vx_{-5.0}, moving_y_{-2.0};
  double static_x_{15.0}, static_y_{3.0};
};

}  // namespace sim_sensors

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sim_sensors::SimSensors>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
