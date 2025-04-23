#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <fstream>
#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <boost/bind/bind.hpp>  // 更新版 boost::bind 输入
using namespace boost::placeholders;  // 避免警告

using sensor_msgs::msg::PointCloud2;

class FusionNode : public rclcpp::Node {
public:
  FusionNode() : Node("fusion_node") {
    this->declare_parameter("save_xyz", false);
    this->declare_parameter("OutExport_path", std::string("/tmp/out.csv"));
    this->declare_parameter("ns_list", std::vector<std::string>{"ns1", "ns2", "ns3", "ns4"});

    save_enabled_ = this->get_parameter("save_xyz").as_bool();
    out_path_ = this->get_parameter("OutExport_path").as_string();
    ns_list_ = this->get_parameter("ns_list").as_string_array();

    fusion_pub_ = this->create_publisher<PointCloud2>("fusion_point", 10);

    for (const auto &ns : ns_list_) {
      std::string param_name = "/" + ns + "/rfans_driver/Is_Start";
      this->declare_parameter(param_name, false);
      bool start = this->get_parameter(param_name).as_bool();
      if (start) {
        active_ns_.push_back(ns);
      }
    }

    if (save_enabled_) {
      file_.open(out_path_);
      file_ << "x,y,z,intent,timeflag,laserid,hangle,mirrorid\n";
    }

    setup_subscribers();
  }

private:
  void callback_pub(const PointCloud2::ConstSharedPtr &p1) {
    publish_and_save({p1});
  }

  void callback_pub(const PointCloud2::ConstSharedPtr &p1, const PointCloud2::ConstSharedPtr &p2) {
    publish_and_save({p1, p2});
  }

  void callback_pub(const PointCloud2::ConstSharedPtr &p1, const PointCloud2::ConstSharedPtr &p2, const PointCloud2::ConstSharedPtr &p3) {
    publish_and_save({p1, p2, p3});
  }

  void callback_pub(const PointCloud2::ConstSharedPtr &p1, const PointCloud2::ConstSharedPtr &p2,
                    const PointCloud2::ConstSharedPtr &p3, const PointCloud2::ConstSharedPtr &p4) {
    publish_and_save({p1, p2, p3, p4});
  }

  void publish_and_save(const std::vector<PointCloud2::ConstSharedPtr>& clouds) {
    if (clouds.empty()) return;

    PointCloud2 fused;
    fused.header.frame_id = "world";
    fused.header.stamp = this->get_clock()->now();
    fused.height = 1;
    fused.point_step = clouds[0]->point_step;
    fused.width = 0;

    size_t total_size = 0;
    for (const auto& c : clouds) {
      fused.width += c->width;
      total_size += c->data.size();
    }
    fused.data.resize(total_size);
    fused.row_step = total_size;

    size_t offset = 0;
    for (const auto& c : clouds) {
      std::copy(c->data.begin(), c->data.end(), fused.data.begin() + offset);
      offset += c->data.size();
    }
    fusion_pub_->publish(fused);

    if (save_enabled_ && file_.is_open()) {
      file_ << "# saved frame with " << fused.width << " points\n";
    }
  }

  void setup_subscribers() {
    using namespace message_filters;

    switch (active_ns_.size()) {
      case 0:
        RCLCPP_WARN(this->get_logger(), "没有启用的雷达节点");
        break;
      case 1:
        sub1_ = std::make_shared<Subscriber<PointCloud2>>(this, "/" + active_ns_[0] + "/lidar_points");
        sub1_->registerCallback(std::bind(static_cast<void(FusionNode::*)(const PointCloud2::ConstSharedPtr&)>(&FusionNode::callback_pub), this, _1));
        break;
      case 2:
        init_sub_sync<2>();
        break;
      case 3:
        init_sub_sync<3>();
        break;
      default:
        init_sub_sync<4>();
        break;
    }
  }

  template<int N>
  void init_sub_sync();

  rclcpp::Publisher<PointCloud2>::SharedPtr fusion_pub_;
  std::vector<std::string> ns_list_;
  std::vector<std::string> active_ns_;
  std::ofstream file_;
  std::string out_path_;
  bool save_enabled_ = false;

  std::shared_ptr<message_filters::Subscriber<PointCloud2>> sub1_, sub2_, sub3_, sub4_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>>> sync2_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2>>> sync3_;
  std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, PointCloud2>>> sync4_;
};

// 模板特化实现

template<>
void FusionNode::init_sub_sync<2>() {
  sub1_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[0] + "/lidar_points");
  sub2_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[1] + "/lidar_points");
  sync2_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>>>(
      message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2>(10), *sub1_, *sub2_);
  sync2_->registerCallback(std::bind(static_cast<void(FusionNode::*)(const PointCloud2::ConstSharedPtr&, const PointCloud2::ConstSharedPtr&)>(&FusionNode::callback_pub), this, _1, _2));
}

template<>
void FusionNode::init_sub_sync<3>() {
  sub1_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[0] + "/lidar_points");
  sub2_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[1] + "/lidar_points");
  sub3_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[2] + "/lidar_points");
  sync3_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2>>>(
      message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2>(10), *sub1_, *sub2_, *sub3_);
  sync3_->registerCallback(std::bind(static_cast<void(FusionNode::*)(const PointCloud2::ConstSharedPtr&, const PointCloud2::ConstSharedPtr&, const PointCloud2::ConstSharedPtr&)>(&FusionNode::callback_pub), this, _1, _2, _3));
}

template<>
void FusionNode::init_sub_sync<4>() {
  sub1_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[0] + "/lidar_points");
  sub2_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[1] + "/lidar_points");
  sub3_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[2] + "/lidar_points");
  sub4_ = std::make_shared<message_filters::Subscriber<PointCloud2>>(this, "/" + active_ns_[3] + "/lidar_points");
  sync4_ = std::make_shared<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, PointCloud2>>>(
      message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2, PointCloud2>(10), *sub1_, *sub2_, *sub3_, *sub4_);
  sync4_->registerCallback(std::bind(static_cast<void(FusionNode::*)(const PointCloud2::ConstSharedPtr&, const PointCloud2::ConstSharedPtr&, const PointCloud2::ConstSharedPtr&, const PointCloud2::ConstSharedPtr&)>(&FusionNode::callback_pub), this, _1, _2, _3, _4));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FusionNode>());
  rclcpp::shutdown();
  return 0;
}