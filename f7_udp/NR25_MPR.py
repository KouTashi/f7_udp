#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
汎用機の機構制御
"""

# Falseにすることでルーター未接続でもデバッグ可能、Trueへの戻し忘れに注意
ONLINE_MODE = True

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from socket import *
import time
import math

# 以下pipでのインストールが必要
import pyfiglet

data = [0, -1, -1, -1, 0, 0, 0, 0, 0]  # 各モーターの出力（0% ~ 100%）

duty_max = 50
sp_yaw = 0.1

deadzone = 0.3  # adjust DS4 deadzone
ready_for_shoot = False


class Listener(Node):

    def __init__(self):
        super().__init__("nhk25_omni_driver")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        print(pyfiglet.figlet_format("NHK2025"))
        self.subscription  # prevent unused variable warning

    def listener_callback(self, ps4_msg):
        LS_X = -1 * ps4_msg.axes[0]
        LS_Y = ps4_msg.axes[1]
        RS_X = (-1) * ps4_msg.axes[2]
        RS_Y = ps4_msg.axes[5]

        # print(LS_X, LS_Y, RS_X, RS_Y)

        CROSS = ps4_msg.buttons[1]
        CIRCLE = ps4_msg.buttons[2]
        TRIANGLE = ps4_msg.buttons[3]
        SQUARE = ps4_msg.buttons[0]

        LEFT = ps4_msg.axes[12] == 1.0
        RIGHT = ps4_msg.axes[12] == -1.0
        UP = ps4_msg.axes[13] == 1.0
        DOWN = ps4_msg.axes[13] == -1.0

        L1 = ps4_msg.buttons[4]
        R1 = ps4_msg.buttons[5]

        L2 = (-1 * ps4_msg.axes[3] + 1) / 2
        R2 = (-1 * ps4_msg.axes[4] + 1) / 2

        SHARE = ps4_msg.buttons[8]
        OPTION = ps4_msg.buttons[9]
        PS = ps4_msg.buttons[12]

        L3 = ps4_msg.buttons[10]
        R3 = ps4_msg.buttons[11]
        
        global ready_for_shoot

        # PSボタンで緊急停止
        if PS:
            print(pyfiglet.figlet_format("HALT"))
            data[1] = -1
            data[2] = -1
            data[3] = -1
            data[4] = 0
            data[5] = 0
            data[6] = 0
            data[7] = 0
            data[8] = 0
            if ONLINE_MODE:
                udp.send()  # 関数実行
            time.sleep(1)
            while True:
                pass

        if CIRCLE and not ready_for_shoot:
            print("Ready for Shooting")
            data[1] = 1
            data[3] = 1 
            data[5] = 30
            data[6] = 30
            if ONLINE_MODE:
                udp.send()  # 関数実行
            CIRCLE = False
            ready_for_shoot = True
            time.sleep(0.5)
            
        if CIRCLE and ready_for_shoot:
            print("Shoot")
            data[2] = 1
            if ONLINE_MODE:
                udp.send()  # 関数実行
            ready_for_shoot = False
            time.sleep(1.0)
            print("Ready for Retraction")
            data[2] = -1
            if ONLINE_MODE:
                udp.send()  # 関数実行
            time.sleep(1.0)
            print("Retract")
            data[1] = -1
            data[3] = -1
            data[5] = 0
            data[6] = 0
            if ONLINE_MODE:
                udp.send()  # 関数実行
                
        if TRIANGLE:
            data[3] = 1
            data[5] = -30
            data[6] = -30
            if ONLINE_MODE:
                udp.send()  # 関数実行
            time.sleep(2.0)
            data[3] = -1
            data[5] = 0
            data[6] = 0
            if ONLINE_MODE:
                udp.send()  # 関数実行
        

        if ONLINE_MODE:
            udp.send()  # 関数実行


class udpsend:
    def __init__(self):

        SrcIP = "192.168.8.196"  # 送信元IP
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.216"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        self.udpClntSock.bind(self.SrcAddr)  # 送信元アドレスでバインド

    def send(self):

        # print(data[1], data[2], data[3], data[4])

        # Duty比のリミッター、消すな！
        for i in range(len(data)):
            if data[i] > duty_max:
                data[i] = duty_max

        str_data = (
            str(data[1])
            + ","
            + str(data[2])
            + ","
            + str(data[3])
            + ","
            + str(data[4])
            + ","
            + str(data[5])
            + ","
            + str(data[6])
            + ","
            + str(data[7])
            + ","
            + str(data[8])
        )  # パケットを作成

        print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        #data[1] = 0
        #data[2] = 0
        #data[3] = 0
        #data[4] = 0
        #data[5] = 0
        #ata[6] = 0
        data[7] = 0
        data[8] = 0


if ONLINE_MODE:
    udp = udpsend()  # クラス呼び出し


def main(args=None):
    rclpy.init(args=args)

    listener = Listener()

    rclpy.spin(listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()