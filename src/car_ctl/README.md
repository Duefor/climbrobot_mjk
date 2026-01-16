# 用于控制底盘的运动
## 26/1/13
也哥的连接can命令：  
//必要  
修改寄存器的值,使能CAN控制器  
sudo busybox devmem 0x0c303018 w 0xc458  
sudo busybox devmem 0x0c303010 w 0xc400  
sudo busybox devmem 0x0c303008 w 0xc458  
sudo busybox devmem 0x0c303000 w 0xc400  

挂载CAN内核  
sudo modprobe can  
sudo modprobe can_raw  
sudo modprobe mttcan  

验证内核是否加载  
lsmod | grep can  

查看接口  
ip link show  

设置接口波特率  
sudo ip link set can0 type can bitrate 500000  

启动 can0  
sudo ip link set up can0  
**写了一个自动化脚本，可以直接开**  

## 26/1/16
