
#include"ioapi.h"
#include"rfans_driver.h"
#include <unistd.h>
#include <string>
#include <sstream>
#include <iostream>
#include <sdk-decoder/Socket.h>
#include <thread>
#include <stdlib.h>
#include<cstdlib>
#include<cmath>
#include <Eigen/Eigen>

using namespace Eigen;
using namespace ss;
namespace rfans_driver
{
size_t packet_size_pcap = 1206;
}
static string out;
std::mutex mutex_;
long tmp_len = sizeof(DEB_FRAME_S);
void udp_write(const std::string& ip, int port, int tmp_len,DEB_FRAME_S wdFrame_)
{
  char buffer[18];
  memset(&buffer, 0, sizeof(buffer));

  Socket udp = Socket::udp();
  if (!udp.connect(InternetEndpoint(ip, port))) {
    std::cerr << "connect failed" << std::endl;
  }

  memcpy(buffer,(char * )&wdFrame_,sizeof (wdFrame_));
  char temp;
  temp=buffer[2];
  buffer[2]=buffer[3];
  buffer[3]=temp;
  int test_number=3;
  while(1){
    if(udp.send(buffer, 18) < 0) {
      std::cout << "error: " << std::endl;
      return;
    }
    usleep(1);
    test_number--;

  }




}

void udp_read(int port)
{
  mutex_.lock();
  char buffer[10];
  Socket udp = Socket::udp();
  if (!udp.bind(InternetEndpoint(port)) ) {
    std::cerr << "connect failed" << std::endl;
  }
  int number=2;
  while (number>1)
  {
    const ssize_t ret = udp.read(buffer, 10);
    if(ret < 0) {
      std::cout << "error: " << std::endl;
      break;
    }
    if((int16_t)buffer[3]==4)
    {
      printf("%02x",(int16_t)buffer[4]);
      printf("%02x",(int16_t)buffer[5]);
      printf("%02x",(int16_t)buffer[6]);
      printf("%02x",(int16_t)buffer[7]);
      printf("\n");
      number--;
    }
  }
  mutex_.unlock();
}





int main(int argc, char **argv)
{

  DEB_FRAME_S wdFrame_;
  wdFrame_ = packDEBV3Frame(eCmdRead, 0x4, 0);
  string ip="192.168.0.3";
  int port_device=2015;
  long tmp_len = sizeof(DEB_FRAME_S);
  thread write_thread(udp_write,ip,port_device,tmp_len,wdFrame_);
  thread read_thread(udp_read,port_device);

  write_thread.join();
  read_thread.join();
  //std::cout<<"output="<<out<<std::endl;
  usleep(5);
  return 1;
}
