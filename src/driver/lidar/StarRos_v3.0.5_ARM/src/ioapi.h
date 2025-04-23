#ifndef __RFANS_IOAPI_H
#define __RFANS_IOAPI_H

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "ssFrameLib.h"
#include <rfans_driver/msg/packet.hpp>
#include <pcap.h>
#include <memory>

class SSBufferDec;

namespace rfans_driver
{

static uint16_t DATA_PORT_NUMBER = 2014;

class IOAPI
{
public:
  IOAPI() = default;
  virtual ~IOAPI() = default;

  virtual int write(unsigned char *data, int size) = 0;
  virtual int read(unsigned char *data, int size) = 0;
  virtual int reset() = 0;
  virtual int HW_WRREG(int flag, int regAddress, unsigned int regData);
};

class IOSocketAPI : public IOAPI
{
public:
  IOSocketAPI(rclcpp::Node::SharedPtr node,
              const std::string &ipstr = DEVICE_IP_STRING,
              uint16_t devport = DATA_PORT_NUMBER,
              int16_t pcport = PC_PORT_NUMBER);
  virtual ~IOSocketAPI();

  virtual int write(unsigned char *data, int size) override;
  virtual int read(unsigned char *data, int size) override;
  virtual int reset() override;

private:
  int m_sockfd;
  sockaddr_in m_devaddr;
  rclcpp::Node::SharedPtr node_;
};

class InputPCAP
{
public:
  InputPCAP(rclcpp::Node::SharedPtr node,
            uint16_t port = DATA_PORT_NUMBER,
            double packet_rate = 0.0,
            const std::string &filename = "",
            const std::string &device_ip = "",
            bool read_once = false,
            bool read_fast = false,
            double repeat_delay = 0.0);

  virtual ~InputPCAP();
  virtual int getPacket(rfans_driver::msg::Packet *pkt);
  void setDeviceIP(const std::string &ip);

private:
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rclcpp::Rate> rate_;
  uint16_t port_;
  std::string devip_str_;
  std::string filename_;
  pcap_t *pcap_;
  struct bpf_program pcap_filter_;
  char errbuf_[PCAP_ERRBUF_SIZE];
  bool empty_;
  bool read_once_;
  bool read_fast_;
  double repeat_delay_;
};

}  // namespace rfans_driver

#endif  // __RFANS_IOAPI_H
