#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// 发送can帧
void sendCanFrame(int s, struct sockaddr_can addr, int can_id, unsigned char data[], int len = 8)
{
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    int nbytes = sendto(s, &frame, sizeof(frame), 0, (struct sockaddr*)&addr, sizeof(addr));
    if (nbytes != sizeof(frame))
        perror("CAN Send Error");
    else{
        std::cout<<"CAN data sent successfully for CAN ID:0x"<<std::hex<<can_id<<std::dec<<std::endl;
    }
    usleep(10000);
}
// 速度转换
void makeSpeedCommand(unsigned char *data, int rpm)
{
    // 对象索引 0x60FF00，速度 = rpm * 500 / 3
    int value = rpm * 500 / 3;
    data[0] = 0x23; data[1] = 0xFF; data[2] = 0x60; data[3] = 0x00;
    memcpy(&data[4], &value, 4);
}
// 初始化can
int setupCanSocket(struct sockaddr_can &addr)
{
    int s;
    struct ifreq ifr;
    const char* ifname = "can0";

    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0)
    {
        perror("Socket creation error");
        return -1;
    }

    strcpy(ifr.ifr_name, ifname);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("SIOCGIFINDEX error");
        close(s);
        return -1;
    }

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind error");
        close(s);
        return -1;
    }

    return s;
}

// 电机初始化：设置模式，加减速度
void motorInit(int s, struct sockaddr_can addr)
{
    unsigned char reset_data[] = {0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00};// 复位节点
    unsigned char remote_control_data[] = {0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00};// 启动节点远程控制
    unsigned char cw_06[] = {0x2b,0x40,0x60,0x00,0x06,0x00,0x00,0x00};// 控制字
    unsigned char cw_07[] = {0x2b,0x40,0x60,0x00,0x07,0x00,0x00,0x00};// 控制字
    unsigned char cw_0f[] = {0x2b,0x40,0x60,0x00,0x0f,0x00,0x00,0x00};// 控制字
    unsigned char speed_mode[] = {0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00};// 速度控制模式03
    unsigned char acc[] = {0x23,0x83,0x60,0x00,0x60,0xE3,0x16,0x00};// 加速度
    unsigned char dec[] = {0x23,0x84,0x60,0x00,0x60,0xE3,0x16,0x00};// 减速度

    ROS_INFO("Reset and start nodes...");
    sendCanFrame(s, addr, 0x0000, reset_data);
    sendCanFrame(s, addr, 0x0000, remote_control_data);

    // 左轮
    ROS_INFO("Config left motor...");
    sendCanFrame(s, addr, 0x601, cw_06);
    sendCanFrame(s, addr, 0x601, cw_07);
    sendCanFrame(s, addr, 0x601, cw_0f);
    sendCanFrame(s, addr, 0x601, speed_mode);
    sendCanFrame(s, addr, 0x601, acc);
    sendCanFrame(s, addr, 0x601, dec);

    // 右轮
    ROS_INFO("Config right motor...");
    sendCanFrame(s, addr, 0x602, cw_06);
    sendCanFrame(s, addr, 0x602, cw_07);
    sendCanFrame(s, addr, 0x602, cw_0f);
    sendCanFrame(s, addr, 0x602, speed_mode);
    sendCanFrame(s, addr, 0x602, acc);
    sendCanFrame(s, addr, 0x602, dec);

    ROS_INFO("Motor initialization done.");
}

// ros回调函数
int g_sock;
struct sockaddr_can g_addr;

void wheelSpeedCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 2) return;

    double left_rpm  = msg->data[0];
    double right_rpm = msg->data[1];

    unsigned char speed_cmd[8];

    makeSpeedCommand(speed_cmd, (int)left_rpm);
    sendCanFrame(g_sock, g_addr, 0x601, speed_cmd);

    makeSpeedCommand(speed_cmd, (int)right_rpm);
    sendCanFrame(g_sock, g_addr, 0x602, speed_cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "can_wheel_controller");
    ros::NodeHandle nh;

    g_sock = setupCanSocket(g_addr);
    if (g_sock < 0)
    {
        ROS_ERROR("CAN socket setup failed.");
        return -1;
    }

    motorInit(g_sock, g_addr);

    ros::Subscriber sub = nh.subscribe("/wheel_speed_cmd", 10, wheelSpeedCallback);
    ROS_INFO("Listening /wheel_speed_cmd ...");

    ros::spin();

    // 停止电机
    unsigned char stop_data[8];
    makeSpeedCommand(stop_data, 0);
    sendCanFrame(g_sock, g_addr, 0x601, stop_data);
    sendCanFrame(g_sock, g_addr, 0x602, stop_data);
    close(g_sock);

    ROS_INFO("Motors stopped. Node exit.");
    return 0;
}
