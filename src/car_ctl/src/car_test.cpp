#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <cstring>

#include <termios.h>
#include <fcntl.h>

using namespace std;

void sendCanFrame(int s, struct sockaddr_can addr, int can_id, unsigned char data[], int data_length = 8){
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = data_length; //根据数据实际长度设置
    memcpy(frame.data,data,data_length);

    int nbytes = sendto(s,&frame,sizeof(frame),0,(struct sockaddr*)&addr,sizeof(addr));
    if(nbytes !=sizeof(frame))
    {
        perror("Send error");
    }
    else{
        cout<<"CAN data sent successfully for CAN ID:0x"<<hex<<can_id<<dec<<endl;
    }
    usleep(10000);
}

// 设置非阻塞输入
void setNonBlockingInput() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~(ICANON | ECHO); // 关闭行缓冲和回显
    ttystate.c_cc[VMIN] = 1; // 每次读取1个字符
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK); // 非阻塞模式
}

int main() {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;
    const char* ifname = "can0";  // 使用 can0 接口

    // 创建套接字
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("Socket creation error");
        return -1;
    }

    // 指定 CAN 接口
    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX error");
        close(s);
        return -1;
    }

    // 填充地址结构体
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind error");
        close(s);
        return -1;
    }

    // 初始化报文数据
    unsigned char reset_data[] = {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  //复位所有节点  ID: 0x0000
    unsigned char remote_control_data[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  //启动所有节点的远程控制  ID: 0x0000
    unsigned char control_word_06h[] = {0x2b, 0x40, 0x60, 0x00, 0x06, 0x00, 0x00, 0x00};  //写控制字为 06H  ID: 0x600+ID --->0x601,0x602.....
    unsigned char control_word_07h[] = {0x2b, 0x40, 0x60, 0x00, 0x07, 0x00, 0x00, 0x00};  //写控制字为 07H  ID: 0x600+ID --->0x601,0x602.....
    unsigned char control_word_0fh[] = {0x2b, 0x40, 0x60, 0x00, 0x0f, 0x00, 0x00, 0x00};  //写控制字为 0FH，电机使能    ID: 0x600+ID --->0x601,0x602.....
    unsigned char speed_control_mode[] = {0x2f, 0x60, 0x60, 0x00, 0x03, 0x00, 0x00, 0x00};  //速度控制模式  ID: 0x600+ID --->0x601,0x602.....
    unsigned char acceleration[] = {0x23, 0x83, 0x60, 0x00, 0x60, 0xE3, 0x16, 0x00};  //加速度111ms/1000rpm ID: 0x600+ID
    unsigned char deceleration[] = {0x23, 0x84, 0x60, 0x00, 0x60, 0xE3, 0x16, 0x00};  //减速度111ms/1000rpm  ID: 0x600+ID
    unsigned char speed_1500rpm[] = {0x23, 0xff, 0x60, 0x00, 0x90, 0xD0, 0x03, 0x00};  //设置速度v=1500r/min,发送数字的大小应该是500*v/3   比如：V为1500rpm/min,需要给电机发送脉冲值大小为1500*500/3=250000-->0x0003D090
    unsigned char speed_0rpm[] = {0x23, 0xff, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};  //设置速度0r/min  ID: 0x600+ID --->0x601,0x602.....
    unsigned char speed_100rpm[] = {0x23, 0xff, 0x60, 0x00, 0x1A, 0x41, 0x00, 0x00};  //设置速度100r/min  ID: 0x600+ID --->0x601,0x602.....
    unsigned char speed_inv1500rpm[] = {0x23, 0xff, 0x60, 0x00, 0x70, 0x2f, 0xfc, 0xff};  //设置速度-1500r/min  ID: 0x600+ID --->0x601,0x602.....

    unsigned char read_request[] = {0x40, 0x6c, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};

    // 依次发送初始化报文
    std::cout << "发送初始化报文" << std::endl;
    sendCanFrame(s, addr, 0x0000, reset_data);
    sendCanFrame(s, addr, 0x0000, remote_control_data);
    std::cout << "初始化完成" << std::endl;
    // 左轮：0x601
    std::cout << "设置左轮参数，进入速度控制模式" << std::endl;
    sendCanFrame(s, addr, 0x601, control_word_06h);
    sendCanFrame(s, addr, 0x601, control_word_07h);
    sendCanFrame(s, addr, 0x601, control_word_0fh);
    sendCanFrame(s, addr, 0x601, speed_control_mode);
    sendCanFrame(s, addr, 0x601, acceleration);
    sendCanFrame(s, addr, 0x601, deceleration);
    // 右轮：0x602
    std::cout << "设置右轮参数，进入速度控制模式" << std::endl;
    sendCanFrame(s, addr, 0x602, control_word_06h);
    sendCanFrame(s, addr, 0x602, control_word_07h);
    sendCanFrame(s, addr, 0x602, control_word_0fh);
    sendCanFrame(s, addr, 0x602, speed_control_mode);
    sendCanFrame(s, addr, 0x602, acceleration);
    sendCanFrame(s, addr, 0x602, deceleration);

    // 实时设置电机速度
    std::cout << "设置左轮速度为1500 r/min" << std::endl;
    sendCanFrame(s, addr, 0x601, speed_1500rpm);
    std::cout << "设置右轮速度为-1500 r/min" << std::endl;
    sendCanFrame(s, addr, 0x602, speed_inv1500rpm);


    // 设置套接字超时
    struct timeval tv;
    tv.tv_sec = 1;  // 1秒超时
    tv.tv_usec = 0;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    setNonBlockingInput(); // 设置非阻塞输入

    while (true) {
        // 读取左轮转速
        sendCanFrame(s, addr, 0x601, read_request);
        struct can_frame response_frame;
        socklen_t len = sizeof(addr);
        int nbytes = recvfrom(s, &response_frame, sizeof(response_frame), 0, (struct sockaddr*)&addr, &len);

        if (nbytes < 0) {
            perror("Read SDO error");
        } else if ((response_frame.can_id == 0x581) && (response_frame.data[0] == 0x43)) {
            uint32_t left_speed = 
                (response_frame.data[4]) |
                (response_frame.data[5] << 8) |
                (response_frame.data[6] << 16) |
                (response_frame.data[7] << 24);
            cout << "Left Motor Speed: " << (int)(left_speed) << " rpm" << endl;
        }

        // 读取右轮转速（同理）
        sendCanFrame(s, addr, 0x602, read_request);  // 请求右轮转速
        nbytes = recvfrom(s, &response_frame, sizeof(response_frame), 0, (struct sockaddr*)&addr, &len);
        if ((response_frame.can_id == 0x582) && (response_frame.data[0] == 0x43)) {
            uint32_t right_speed =
                (response_frame.data[4]) |
                (response_frame.data[5] << 8) |
                (response_frame.data[6] << 16) |
                (response_frame.data[7] << 24);  // 解析数据
            cout << "Right Motor Speed: " << (int)(right_speed) << " rpm" << endl;
        }

        // 检查键盘输入
        char input;
        if (read(STDIN_FILENO, &input, 1) > 0 && input == 'q') {
            break; // 退出循环
        }

        usleep(100000);  // 100ms 读取一次
    }
    

    // 设置电机速度为0实现停止
    std::cout << "停止电机" << std::endl;
    sendCanFrame(s, addr, 0x601, speed_0rpm);
    sendCanFrame(s, addr, 0x602, speed_0rpm);

    close(s);
    return 0;
  
}