#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// 发送 CAN 帧
void sendCanFrame(int s, struct sockaddr_can addr, int can_id, unsigned char data[8]) {
    struct can_frame frame;
    frame.can_id = can_id;
    frame.can_dlc = 8;
    for (int i = 0; i < 8; i++) {
        frame.data[i] = data[i];
    }
    int nbytes = write(s, &frame, sizeof(frame));
    if (nbytes!= sizeof(frame)) {
        perror("Write error");
    } else {
        std::cout << "CAN data sent successfully for CAN ID: 0x" << std::hex << can_id << std::dec << std::endl;
    }
    // 添加 10 毫秒的延迟
    usleep(10000); 
}

// 将整数转换为CAN报文格式，后面四个为数据域
void int32ToCanData(int32_t value, unsigned char* data, int start_index) {
    data[start_index] = value & 0xFF;
    data[start_index + 1] = (value >> 8) & 0xFF;
    data[start_index + 2] = (value >> 16) & 0xFF;
    data[start_index + 3] = (value >> 24) & 0xFF;
    std::cout << data[start_index] << "," << data[start_index + 1] << "," << data[start_index + 2] << "," << data[start_index + 3] << std::endl;
}

// 读取 CAN 电机状态字
int32_t readMotorStatus(int s, int can_id) {
    struct can_frame frame;
    int nbytes = read(s, &frame, sizeof(frame));
    if (nbytes < 0) {
        perror("CAN read error");
        return -1;
    }
    if (frame.can_id == can_id && frame.can_dlc >= 6 && frame.data[1] == 0x41 && frame.data[2] == 0x60) {
        // status word 在后两字节
        int16_t status = frame.data[4] | (frame.data[5] << 8);
        return status;
    }
    return -1;  // 非目标帧
}

bool waitMotorFinish(int s, int can_id, int timeout_ms = 5000) {
    const int interval_us = 10000; // 10ms
    int elapsed = 0;
    while (elapsed < timeout_ms * 1000) {
        int status = readMotorStatus(s, can_id);
        if (status >= 0) {
            // 根据 CiA402 标准，bit 0 = Ready to switch on, bit 2 = Switch on, bit 3 = Fault, bit 12 = Target reached
            if (status & (1 << 10)) { // bit 12 = Target reached
                std::cout << "Motor 0x" << std::hex << can_id << " has finished moving." << std::dec << std::endl;
                return true;
            }
        }
        usleep(interval_us);
        elapsed += interval_us;
    }
    std::cout << "Timeout waiting for motor 0x" << std::hex << can_id << std::dec << std::endl;
    return false;
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
    unsigned char position_control_mode[] = {0x2f, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00};  //位置控制模式  ID: 0x600+ID --->0x601,0x602.....
    unsigned char acceleration[] = {0x23, 0x83, 0x60, 0x00, 0x90, 0xd0, 0x03, 0x00};  //加速度1500r/min ID: 0x600+ID
    unsigned char deceleration[] = {0x23, 0x84, 0x60, 0x00, 0x90, 0xd0, 0x03, 0x00};  //减速度1500r/min  ID: 0x600+ID
    unsigned char speed_1500rpm[] = {0x23, 0x81, 0x60, 0x00, 0x90, 0xd0, 0x03, 0x00};  //设置速度v=1500r/min,发送数字的大小应该是500*v/3   比如：V为1500rpm/min,需要给电机发送脉冲值大小为1500*500/3=250000-->0x0003D090
    unsigned char rela_mode[] = {0x2b, 0x40, 0x60, 0x00, 0x4f, 0x00, 0x00, 0x00};  // 设置为相对位置模式 (6040h = 4Fh)
    unsigned char start_move[] = {0x2b, 0x40, 0x60, 0x00, 0x5f, 0x00, 0x00, 0x00};  // 开始运动 (6040h = 5Fh)

    // 依次发送初始化报文
    std::cout << "发送初始化报文" << std::endl;
    sendCanFrame(s, addr, 0x0000, reset_data);
    sendCanFrame(s, addr, 0x0000, remote_control_data);
    std::cout << "初始化完成" << std::endl;
    // 左轮：0x601
    std::cout << "设置左轮参数，进入位置控制模式" << std::endl;
    sendCanFrame(s, addr, 0x601, control_word_06h);
    sendCanFrame(s, addr, 0x601, control_word_07h);
    sendCanFrame(s, addr, 0x601, control_word_0fh);
    sendCanFrame(s, addr, 0x601, position_control_mode);
    sendCanFrame(s, addr, 0x601, acceleration);
    sendCanFrame(s, addr, 0x601, deceleration);
    sendCanFrame(s, addr, 0x601, speed_1500rpm);
    // 右轮：0x602
    std::cout << "设置右轮参数，进入位置控制模式" << std::endl;
    sendCanFrame(s, addr, 0x602, control_word_06h);
    sendCanFrame(s, addr, 0x602, control_word_07h);
    sendCanFrame(s, addr, 0x602, control_word_0fh);
    sendCanFrame(s, addr, 0x602, position_control_mode);
    sendCanFrame(s, addr, 0x602, acceleration);
    sendCanFrame(s, addr, 0x602, deceleration);
    sendCanFrame(s, addr, 0x602, speed_1500rpm);

    // 减速比500
    int G = 500;
    const int PULSES_PER_REVOLUTION = 10000;  // 10000 PP/r，应该是现在设置的默认值

    // 左轮目标转动圈数
    float num_revolutions = -0.25;
    // 所需的脉冲数
    int32_t target_position = num_revolutions * PULSES_PER_REVOLUTION * G;  //2 * 10000 * 减速比
    // 设置目标位置 (607AH)，后四位为待定的目标位置每秒脉冲数
    unsigned char pos_data[8] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32ToCanData(target_position, pos_data, 4);
    sendCanFrame(s, addr, 0x601, pos_data);
    // 设置为相对位置模式 (6040h = 4Fh)
    sendCanFrame(s, addr, 0x601, rela_mode);
    // 启动运动 (6040h = 5Fh)
    sendCanFrame(s, addr, 0x601, start_move);
    // 等待左轮完成
    // waitMotorFinish(s, 0x581);  // 0x600 + 0x01的返回报文ID通常是0x581

    // 右轮目标转动圈数
    float num_revolutions1 = -0.25;
    // 所需的脉冲数
    int32_t target_position1 = num_revolutions1 * PULSES_PER_REVOLUTION * G;  //2 * 10000 * 减速比
    // 设置目标位置 (607AH)，后四位为待定的目标位置每秒脉冲数
    unsigned char pos_data1[8] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
    int32ToCanData(target_position1, pos_data1, 4);
    sendCanFrame(s, addr, 0x602, pos_data1);
    // 设置为相对位置模式 (6040h = 4Fh)
    sendCanFrame(s, addr, 0x602, rela_mode);
    // 启动运动 (6040h = 5Fh)
    sendCanFrame(s, addr, 0x602, start_move);
    // 等待右轮完成
    // waitMotorFinish(s, 0x582);  // 0x600 + 0x02的返回报文ID通常是0x582

    close(s);
    return 0;
}