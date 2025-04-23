// /* -*- mode: C++ -*-
//  *  All right reserved, Sure_star Coop.
//  *  @Technic Support: <sdk@isurestar.com>
//  *  $Id$
//  */

// #include <unistd.h>
// #include <string>
// #include <sstream>
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <poll.h>
// #include <errno.h>
// #include <fcntl.h>
// #include <sys/file.h>
// #include "ioapi.h"
// //#include "bufferDecode.h"

// namespace rfans_driver
// {

// static const int POLL_TIMEOUT = 1000; // one second (in msec)
// static const size_t packet_size = sizeof(rfans_driver::msg::RfansPacket().data);
// static size_t packet_size_pcap=1206;

// ////////////////////////////////////////////////////////////////////////
// // base class implementation
// ////////////////////////////////////////////////////////////////////////
// IOAPI::IOAPI()
// {
// }

// IOAPI::~IOAPI() {
// }

// /** @brief Write the Regidit. */
// int IOAPI::HW_WRREG(int flag, int regAddress, unsigned int regData) {
//   int rtn = 0;
//   lidarAPi::DEB_FRAME_S wdFrame_;
//   long tmp_len = sizeof(lidarAPi::DEB_FRAME_S);
//   //ROS_INFO("write the regidit length=%d",tmp_len);
//   wdFrame_ = packDEBV3Frame(lidarAPi::eCmdWrite, regAddress, regData);
//   write((unsigned char *)&wdFrame_, tmp_len);
//   return rtn;
// }


// ////////////////////////////////////////////////////////////////////////
// // InputSocket class implementation
// ////////////////////////////////////////////////////////////////////////
// /** @brief constructor
//          *
//          *  @param private_nh ROS handle for calling node.
//          *  @param port UDP port number
//          */
// IOSocketAPI::IOSocketAPI(std::string ipstr, uint16_t devport, int16_t pcport)
// {
//   m_sockfd = -1;
//   m_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
//   //ROS_INFO("m_devaddr6=%d",m_sockfd);
//   if (m_sockfd == -1) {
//     perror(" create socket error");
//     return;
//   }

//   sockaddr_in my_addr;
//   memset(&my_addr, 0, sizeof(my_addr));
//   my_addr.sin_family = AF_INET;
//   my_addr.sin_port = htons(pcport);
//   my_addr.sin_addr.s_addr = INADDR_ANY;

//   memset(&m_devaddr, 0, sizeof(m_devaddr));
//   m_devaddr.sin_family = AF_INET;
//   m_devaddr.sin_port = htons(devport);
//   m_devaddr.sin_addr.s_addr = inet_addr(ipstr.c_str() );

//   if (bind(m_sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) {
//     perror("bind message");
//     return;
//   }
//   int buff_size=2000000;
//   setsockopt(m_sockfd,SOL_SOCKET,SO_RCVBUF,(const char*)(&buff_size),sizeof (int));
//   if (fcntl(m_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
//     perror("non-block message");
//     return;
//   }

//   //ROS_INFO_STREAM("rfans socket fd is " << m_sockfd);
//   reset();
// }

// /** @brief destructor */
// IOSocketAPI::~IOSocketAPI(void)
// {
//   (void)close(m_sockfd);
//   //ROS_INFO("m_devaddr7=%d",m_sockfd);
// }

// /** @brief Write */
// int IOSocketAPI::write(unsigned char *data, int size)
// {
//   int rtn = 0 ;
//   unsigned char tmpCmd[UDP_FRAME_MIN] ;
//   unsigned char *tmpBuf = data ;
//   socklen_t sender_address_len = sizeof(m_devaddr);
//   //ROS_INFO("m_devaddr1=%d",m_sockfd);
//   if(m_sockfd <= 0) return rtn ;

//   if (size < UDP_FRAME_MIN && size > 0) {
//     memcpy(tmpCmd, data, size);
//     tmpBuf = tmpCmd;
//     size = UDP_FRAME_MIN;
//   }
//  // ROS_INFO("write size=%d",size);
//   rtn = sendto(m_sockfd, data, size, 0, (sockaddr*)&m_devaddr, sender_address_len ) ;
//   if (rtn < 0) {
//     //perror("IOSocketAPI:write") ;
//   }
//   return rtn ;
// }

// int IOSocketAPI::reset()
// {
// //  m_bufferPro->reset();
//   return 0;
// }

// /** @brief read from RFans UDP Data  */
// int IOSocketAPI::read(unsigned char *data, int size)
// {
//   struct pollfd fds[1];
//   fds[0].fd = m_sockfd;
//   fds[0].events = POLLIN;
//   int nbytes = 0 ;
//   sockaddr_in sender_address;
//   socklen_t sender_address_len = sizeof(sender_address);

//   int retval = 0 ;
//   retval = poll(fds, 1, POLL_TIMEOUT);
//   if(fds[0].revents & POLLIN) {
//     nbytes = recvfrom(m_sockfd, data, size, 0,
//                       (sockaddr*)&sender_address, &sender_address_len);
//     if(m_devaddr.sin_addr.s_addr != sender_address.sin_addr.s_addr) {
//       nbytes = 0 ;
//     }
//   }
//   if (retval == 0) {
//     ROS_WARN("IOSocketAPI::read  poll() timeout");
//     nbytes = 0;
//   }
//   return nbytes ;
// }
// ////////////////////////////////////////////////////////////////////////

//     InputPCAP::InputPCAP(ros::NodeHandle private_nh, uint16_t port,
//                           double packet_rate,std::string filename,
//                          std::string device_ip,bool read_once,
//                          bool read_fast, double repeat_delay):
//        private_nh_(private_nh),
//        port_(port),
//        packet_rate_(packet_rate),
//        filename_(filename),
//        devip_str_(device_ip)
//      {
//        pcap_ = NULL;
//        empty_ = true;

//        // get parameters using private node handle
//        private_nh.param("read_once", read_once_, false);
//        private_nh.param("read_fast", read_fast_, false);
//        private_nh.param("repeat_delay", repeat_delay_, 0.0);

//        if (read_once_)
//          ROS_INFO("Read input file only once.");
//        if (read_fast_)
//          ROS_INFO("Read input file as quickly as possible.");
//        if (repeat_delay_ > 0.0)
//          ROS_INFO("Delay %.3f seconds before repeating input file.",
//                   repeat_delay_);

//        // Open the PCAP dump file
//        ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
//        if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_) ) == NULL)
//          {
//            ROS_FATAL("Error opening Rfans socket dump file, please set a correct path.");
//            return;
//          }

//        std::stringstream filter;
//        if( devip_str_ != "" )              // using specific IP?
//          {
//            filter << "src host " << devip_str_ << " && ";
//          }
//        filter << "udp dst port " << port;
//        pcap_compile(pcap_, &pcap_packet_filter_,
//                     filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
//      }

//      /** destructor */
//      InputPCAP::~InputPCAP(void)
//      {
//        pcap_close(pcap_);
//      }

//      /** @brief Get one R-Fans packet. */
//      int InputPCAP::getPacket(rfans_driver::msg::Packet *pkt)
//      {
//        struct pcap_pkthdr *header;
//        const u_char *pkt_data;

//        while (true)
//          {
//            int res;
//            if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
//              {
//                // Skip packets not for the correct port and from the
//                // selected IP address.
//                if (0 == pcap_offline_filter(&pcap_packet_filter_,
//                                              header, pkt_data))
//                  continue;

//                // Keep the reader from blowing through the file.
//                if (read_fast_ == false)
//                  packet_rate_.sleep();
//                packet_size_pcap = header->caplen - 42;
//                pkt->data.resize(packet_size_pcap);
//                memcpy(&pkt->data[0], pkt_data+42, packet_size_pcap);
//                pkt->stamp = ros::Time::now(); // time_offset not considered here, as no synchronization required
//                empty_ = false;
//                return 0;                   // success
//              }

//            if (empty_)                 // no data in file?
//              {
//                ROS_WARN("Error %d reading R-Fans packet: %s",
//                         res, pcap_geterr(pcap_));
//                return -1;
//              }

//            if (read_once_)
//              {
//                ROS_INFO("end of file reached -- done reading.");
//                return -1;
//              }

//            if (repeat_delay_ > 0.0)
//              {
//                ROS_INFO("end of file reached -- delaying %.3f seconds.",
//                         repeat_delay_);
//                usleep(rint(repeat_delay_ * 1000000.0));
//              }

//            ROS_DEBUG("replaying R-Fans dump file");

//            // I can't figure out how to rewind the file, because it
//            // starts with some kind of header.  So, close the file
//            // and reopen it with pcap.
//            pcap_close(pcap_);
//            pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
//            empty_ = true;              // maybe the file disappeared?
//          } // loop back and try again
//      }



// }//end namespace
/* -*- mode: C++ -*-
 *  Converted to ROS 2
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

 #include <unistd.h>
 #include <string>
 #include <sstream>
 #include <sys/socket.h>
 #include <arpa/inet.h>
 #include <poll.h>
 #include <errno.h>
 #include <fcntl.h>
 #include <sys/file.h>
 #include <pcap/pcap.h>
 
 #include <rclcpp/rclcpp.hpp>
 #include "rfans_driver/msg/packet.hpp"
 #include "ioapi.h"
 
 namespace rfans_driver
 {
 
 int IOAPI::HW_WRREG(int flag, int regAddress, unsigned int regData) {
   lidarAPi::DEB_FRAME_S wdFrame_ = packDEBV3Frame(
     lidarAPi::eCmdWrite, regAddress, regData);
   return write(reinterpret_cast<unsigned char*>(&wdFrame_),
                static_cast<int>(sizeof(wdFrame_)));
 }
 
 static const int POLL_TIMEOUT = 1000;
 
 IOSocketAPI::IOSocketAPI(rclcpp::Node::SharedPtr node,
                          const std::string &ipstr,
                          uint16_t devport,
                          int16_t pcport)
   : node_(node)
 {
   m_sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
   if (m_sockfd < 0) {
     RCLCPP_ERROR(node_->get_logger(), "创建 socket 失败：%s", strerror(errno));
     return;
   }
 
   sockaddr_in my_addr{};
   my_addr.sin_family = AF_INET;
   my_addr.sin_port = htons(pcport);
   my_addr.sin_addr.s_addr = INADDR_ANY;
   if (::bind(m_sockfd, reinterpret_cast<sockaddr*>(&my_addr), sizeof(my_addr)) < 0) {
     RCLCPP_ERROR(node_->get_logger(), "bind 失败：%s", strerror(errno));
     return;
   }
 
   memset(&m_devaddr, 0, sizeof(m_devaddr));
   m_devaddr.sin_family = AF_INET;
   m_devaddr.sin_port = htons(devport);
   m_devaddr.sin_addr.s_addr = inet_addr(ipstr.c_str());
 
   int buff_size = 2 * 1000 * 1000;
   ::setsockopt(m_sockfd, SOL_SOCKET, SO_RCVBUF, &buff_size, sizeof(buff_size));
 
   if (::fcntl(m_sockfd, F_SETFL, O_NONBLOCK | FASYNC) < 0) {
     RCLCPP_WARN(node_->get_logger(), "设置非阻塞失败：%s", strerror(errno));
   }
 
   RCLCPP_INFO(node_->get_logger(), "UDP socket 初始化完成，fd=%d", m_sockfd);
 }
 
 IOSocketAPI::~IOSocketAPI() {
   if (m_sockfd >= 0) ::close(m_sockfd);
 }
 
 int IOSocketAPI::write(unsigned char *data, int size) {
   if (m_sockfd < 0) return -1;
   int send_len = ::sendto(m_sockfd, data, size, 0,
                           reinterpret_cast<sockaddr*>(&m_devaddr),
                           sizeof(m_devaddr));
   if (send_len < 0) {
     RCLCPP_WARN(node_->get_logger(), "sendto 失败：%s", strerror(errno));
   }
   return send_len;
 }
 
 int IOSocketAPI::read(unsigned char *data, int size) {
   struct pollfd pfd{m_sockfd, POLLIN, 0};
   int ret = ::poll(&pfd, 1, POLL_TIMEOUT);
   if (ret > 0 && (pfd.revents & POLLIN)) {
     sockaddr_in sender{};
     socklen_t len = sizeof(sender);
     int n = ::recvfrom(m_sockfd, data, size, 0,
                        reinterpret_cast<sockaddr*>(&sender), &len);
     if (n > 0 && sender.sin_addr.s_addr != m_devaddr.sin_addr.s_addr) {
       return 0;
     }
     return n;
   }
   if (ret == 0) {
     RCLCPP_WARN(node_->get_logger(), "读取超时 %d ms", POLL_TIMEOUT);
   }
   return 0;
 }
 
 int IOSocketAPI::reset() {
   return 0;
 }
 
 InputPCAP::InputPCAP(rclcpp::Node::SharedPtr node,
                      uint16_t port,
                      double packet_rate,
                      const std::string &filename,
                      const std::string &device_ip,
                      bool read_once,
                      bool read_fast,
                      double repeat_delay)
   : node_(node),
     port_(port),
     devip_str_(device_ip),
     filename_(filename),
     read_once_(read_once),
     read_fast_(read_fast),
     repeat_delay_(repeat_delay)
 {
   node_->declare_parameter<bool>("read_once", false);
   node_->declare_parameter<bool>("read_fast", false);
   node_->declare_parameter<double>("repeat_delay", 0.0);
 
   node_->get_parameter("read_once", read_once_);
   node_->get_parameter("read_fast", read_fast_);
   node_->get_parameter("repeat_delay", repeat_delay_);
 
   RCLCPP_INFO(node_->get_logger(), "打开 PCAP 文件：%s", filename.c_str());
   pcap_ = pcap_open_offline(filename.c_str(), errbuf_);
   if (!pcap_) {
     RCLCPP_FATAL(node_->get_logger(), "无法打开 PCAP：%s", errbuf_);
     empty_ = true;
     return;
   }
 
   std::stringstream filt;
   if (!device_ip.empty()) {
     filt << "src host " << device_ip << " && ";
   }
   filt << "udp dst port " << port;
   if (pcap_compile(pcap_, &pcap_filter_, filt.str().c_str(), 1, PCAP_NETMASK_UNKNOWN) < 0) {
     RCLCPP_WARN(node_->get_logger(), "pcap_compile 失败：%s", pcap_geterr(pcap_));
   }
 
   rate_ = std::make_unique<rclcpp::Rate>(packet_rate);
   empty_ = false;
 }
 
 InputPCAP::~InputPCAP() {
   if (pcap_) pcap_close(pcap_);
 }
 
 int InputPCAP::getPacket(rfans_driver::msg::Packet *pkt) {
   struct pcap_pkthdr *hdr;
   const u_char *data;
   while (true) {
     int res = pcap_next_ex(pcap_, &hdr, &data);
     if (res >= 0) {
       if (pcap_offline_filter(&pcap_filter_, hdr, data) == 0) continue;
       if (!read_fast_) rate_->sleep();
 
       size_t ps = hdr->caplen - 42;
       pkt->data.resize(ps);
       memcpy(pkt->data.data(), data + 42, ps);
       pkt->stamp = node_->get_clock()->now();
       return 0;
     }
 
     if (empty_ || read_once_) {
       RCLCPP_INFO(node_->get_logger(), "PCAP 回放结束");
       return -1;
     }
 
     if (repeat_delay_ > 0.0) {
       usleep(static_cast<useconds_t>(repeat_delay_ * 1e6));
     }
 
     RCLCPP_INFO(node_->get_logger(), "PCAP 重播中...");
     pcap_close(pcap_);
     pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
     empty_ = false;
   }
 }
 
 void InputPCAP::setDeviceIP(const std::string &ip) {
   devip_str_ = ip;
 }
 
 }  // namespace rfans_driver
 