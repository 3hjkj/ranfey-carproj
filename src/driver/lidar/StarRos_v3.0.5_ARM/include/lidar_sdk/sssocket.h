#ifndef SSSOCKET_H
#define SSSOCKET_H
#include <string>
//#include "sssocket_global.h"
#if defined(_WIN32)
#include <Winsock2.h>
#include <WS2tcpip.h>
#include <MSTcpIp.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/un.h>
#include <linux/tcp.h>
#include <unistd.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include<fcntl.h>
#include<sys/types.h>
#include<sys/ioctl.h>
#include<dirent.h>
#endif
#include<string>
#include"compile.h"
//网络接口前面加WSA为Winsock2的函数


class __dllexport SsSocket
{
public:
    typedef int state_t;
    constexpr static state_t NONE = 0;
    constexpr static state_t CLOSED = -2;
    constexpr static state_t OPENED = 1;
    constexpr static state_t BINDED = 2;
    constexpr static state_t CONNECTED = 4;
    enum SOCKET_TYPE
    {
        UDP_CLIENT=1,
        UDP_SERVER=2,
        TCP_CLIENT=3,
        TCP_SERVER=4,
    };
#if defined(_WIN32)
    typedef int socklen_t;
#ifdef _WIN64
    typedef unsigned __int64 socket_t;
#else
    typedef uint32_t     socket_t;
#endif
    //    typedef void *HANDLE;
    //    typedef HANDLE handle_t;
    typedef socket_t handle_t;
#else
    typedef int     socket_t;
    typedef socket_t handle_t;
#endif
public:

    static SsSocket udp_client_ipv6();
    static SsSocket udp_server_ipv6();
    static SsSocket tcp_client_ipv6();
    static SsSocket tcp_server_ipv6();

	static SsSocket udp_client_ipv4();
	static SsSocket udp_server_ipv4();
	static SsSocket tcp_client_ipv4();
	static SsSocket tcp_server_ipv4();


    SsSocket()=delete ;
    SsSocket(const SsSocket &)=delete ;
    SsSocket &operator=( SsSocket &other) ;


    SsSocket(SsSocket&& other) noexcept;
    SsSocket& operator=(SsSocket&& other) noexcept;

    /**
    * 构造函数
    * Socket构造函数不会执行socket创建套接字，创建套接字应该执行Socket::socket函数
    * @param family socket的family
    * @param type socket的type
    * @param protocol socket的protocol
    */
    SsSocket(int family, int type, int protocol,SOCKET_TYPE socket_type);


	bool bind(std::string ip, int dstPort, int localPort);

	/**
	* 绑定socket，绑定对方主机ip地址和端口，以及自身端口
	* @param address 绑定端口（常用于接收数据使用）
	*/
	bool bind(std::string ip, int dstPort,string local_ip, int localPort);

    /**
    * 绑定socket,由于未绑定自身端口，只能发出数据，不能接收数据
    * @param address 绑定端口（常用于接收数据使用）
    */
    bool bind(std::string ip, int dstPort);


    /**
    * 绑定socket
    * @param Port 绑定端口（常用于绑定udp服务端）
    */
    bool bind(int Port);


    /**
    * 服务端：监听socket 服务器调用bind时候绑定了服务器要监听的ip和端口。
    * @param count 最大允许监听socket的个数
    */
    bool listen();


    /**
    * 服务端：接收客户端的通信。
    */
    bool accept() ;


    /**
    * 客户端：连接到服务器
    */
    bool connect(const std::string& ip,int port);


    /**
    * 服务端：监听socket 服务器调用bind时候绑定了服务器要监听的ip和端口。
    * @param count 最大允许监听socket的个数
    */
    long read(char* buffer, size_t length);



    long write(const char* buffer, size_t length);


    /**
    * 接收
    * @param buffer 接收缓存
    * @param len 接收缓存的长度
    * @param flags 标记，默认为0
    * @return 实际接收的长度
    */
    long receive(char* buffer, size_t length, int flags=0);




    long send(const char* buffer, size_t length, int flags=0);


    /**
    * 断开链接
    */
    void disconnect();

    /**
    * 断开链接,释放相关资源；
    */
    void close();

    /**
    * 等同于setsockopt()
    * @param level 等级，当对socket接口设置时，该参数指定为SOL_SOCKET
    * @param option 选项名称
    * @param value 设置参数的地址
    * @param length 设置参数的长度
    */
    bool setOption(socket_t& socket_,int level, int option, const void *value, int length);

    /**
    * 获取打开状态
    * @return socket已经创建则返回true，否则返回false
    */
    bool opened() const;


    /**
       * 获取绑定状态
       * @return socket已经绑定则返回true，否则返回false
       */
    bool binded() const;

    /**
        * 获取连接状态
        * @return socket已经连接成功则返回true，否则返回false
        */
    bool connected() const;


    /**
        * 获取接收缓存数据长度
        * @return 接收缓存中可用的字节数
        */
    int available();


    /**
    * Socket阻塞模式设置
    * @param true 设置为非阻塞模式，false 设置为阻塞模式
    */
    bool setNonblocking(bool nonblocking);


    /**
    * Socket 接收发送超时设置
    * @param time 超时时间
    */
    bool setTimeout(unsigned long time);


private:
    //禁用拷贝



    bool ioctl(long cmd, unsigned long *arg);
protected:
    socket_t        m_socket_local;//UDP收发的socket
    socket_t         m_socket_device;//TCP用于接收数据使用
    int             _family;
    int             _type;
    int             _protocol;

    state_t         _state;

    sockaddr_in m_addr_device;//设备的地址信息
    sockaddr_in m_addr_local;//本机的地址信息

	sockaddr_in6 m_addr_device_ip6;//设备的地址信息
	sockaddr_in6 m_addr_local_ip6;//本机的地址信息
    SOCKET_TYPE m_socket_type;
	bool is_ipv6;

};

#endif // SSSOCKET_H
