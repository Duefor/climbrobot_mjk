#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import rospy


class EliteRobotClient:
    def __init__(self, ip="192.168.1.10", port=30001):
        self.ip = ip
        self.port = port
        self.sock = None

    def connect(self):
        """连接到机器人控制器"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(3)
            self.sock.connect((self.ip, self.port))
            rospy.loginfo("Connected to robot %s:%d", self.ip, self.port)
        except Exception as e:
            rospy.logerr("Failed to connect to robot: %s", e)
            self.sock = None

    def send_script(self, script: str):
        """发送 EliteScript 脚本"""
        if self.sock is None:
            self.connect()

        try:
            self.sock.sendall(script.encode("utf-8"))
            rospy.loginfo("Force reading loop script sent to robot.")
        except Exception as e:
            rospy.logerr("Error sending script: %s", e)

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None


def build_force_loop_script():
    script = """
def run_func():
    zero_ftsensor()
    sleep(1)

    def read_force_loop():
        while True:
            ft = get_tcp_force(True)

            textmsg("FT vector:", ft)
            sleep(0.05)
        end
    end
    read_force_loop()
end
"""
    return script


def main():
    rospy.init_node("force_read_loop_sender")

    # 修改成你机械臂的 IP
    robot_ip = rospy.get_param("~robot_ip", "192.168.1.200")

    client = EliteRobotClient(ip=robot_ip)

    script = build_force_loop_script()
    client.send_script(script)

    rospy.loginfo("Force read loop started on robot.")

    rospy.spin()  # 保持节点运行，不退出
    client.close()


if __name__ == "__main__":
    main()
