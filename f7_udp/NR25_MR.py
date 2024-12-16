#!/usr/bin/env python3
## coding: UTF-8

"""
RRST NHK2025
汎用機の機構制御

2024/12/13: クラス名の変更、機構制御の関数化、GUI連携、大規模な修正のため実機テスト注意
2024/12/16: 配列dataの拡張
TODO: MD出力のPublish(デバッグ用)
TODO: ds4drvの代替手段の選定
TODO: pyfiglet未インストール時の例外処理
"""

# Falseにすることでルーター未接続でもデバッグ可能、Trueへの戻し忘れに注意
# 実装済み: アドレスのバインドに失敗すると自動でオフラインモードで開始される
ONLINE_MODE = True

# パラメーター調整モード、GUIでのパラメーター変更を有効化
GUI_PARAM_MODE = True

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray

from socket import *
import time
import math

# 以下pipでのインストールが必要
import pyfiglet

data = [
    0,  # 未使用、送信もされないので注意
    0,  # MD1(0% ~ 100%)
    0,  # MD2(0% ~ 100%)
    0,  # MD3(0% ~ 100%)
    0,  # MD4(0% ~ 100%)
    0,  # MD5(0% ~ 100%)
    0,  # MD6(0% ~ 100%)
    0,  # MD7(0% ~ 100%)
    0,  # MD8(0% ~ 100%)
    -1,  # サーボ1 or 電磁弁1(0 ~ 360)or(0,1)
    -1,  # サーボ2 or 電磁弁2(0 ~ 360)or(0,1)
    -1,  # サーボ3 or 電磁弁3(0 ~ 360)or(0,1)
    -1,  # サーボ4 or 電磁弁4(0 ~ 360)or(0,1)
    0,  # パイロットランプ1(0,1)
    0,  # パイロットランプ2(0,1)
    0,  # その他通信(0 ~ 999)
    0,  # その他通信(0 ~ 999)
]

"""
割り当て表
data[0] 	N/A
data[1] 	MD1
data[2] 	MD2
data[3] 	MD3
data[4] 	MD4
data[5] 	MD5
data[6] 	MD6
data[7] 	未割当
data[8] 	未割当
data[9] 	電磁弁1
data[10] 	電磁弁2
data[11] 	電磁弁3
data[12] 	未割当
data[13] 	未割当
data[14] 	未割当
data[15] 	未割当
data[16] 	未割当
"""

duty_max = 80
sp_yaw = 0.1

deadzone = 0.3  # Adjust DS4 deadzone
ready_for_shoot = False

roller_speed_dribble_ab = 20
roller_speed_dribble_cd = 60
roller_speed_shoot_ab = 50
roller_speed_shoot_cd = 50
shoot = 0
dribble = 0


class Listener(Node):

    def __init__(self):
        super().__init__("nhk25_mr")
        self.subscription = self.create_subscription(
            Joy, "joy", self.listener_callback, 10
        )
        print(pyfiglet.figlet_format("MR"))
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
            data[1] = 0
            data[2] = 0
            data[3] = 0
            data[4] = 0
            data[5] = 0
            data[9] = -1
            data[10] = -1
            data[11] = -1
            udp.send()  # UDPで送信
            time.sleep(1)
            while True:
                pass

        # 射出準備
        if CIRCLE and not ready_for_shoot:
            Action.ready_for_shoot()
            CIRCLE = False
            time.sleep(0.5)

        # 射出シーケンス
        if CIRCLE and ready_for_shoot:
            Action.shoot()

        # ドリブル
        if TRIANGLE and not ready_for_shoot:
            Action.dribble()

        udp.send()  # UDPで送信


class Param_Listener(Node):

    def __init__(self):
        super().__init__("param_listener")
        self.subscription = self.create_subscription(
            Int32MultiArray, "parameter_array", self.param_listener_callback, 10
        )
        self.subscription  # prevent unused variable warning

    def param_listener_callback(self, gui_msg):

        global roller_speed_dribble_ab
        global roller_speed_dribble_cd
        global roller_speed_shoot_ab
        global roller_speed_shoot_cd
        global shoot
        global dribble

        roller_speed_dribble_ab = gui_msg.data[0]
        roller_speed_dribble_cd = gui_msg.data[1]
        roller_speed_shoot_ab = gui_msg.data[2]
        roller_speed_shoot_cd = gui_msg.data[3]
        shoot = gui_msg.data[4]
        dribble = gui_msg.data[5]

        global ready_for_shoot

        # 射出準備
        if shoot and not ready_for_shoot:
            Action.ready_for_shoot()
            shoot = 0
            time.sleep(0.5)

        # 射出シーケンス
        if shoot and ready_for_shoot:
            Action.shoot()

        # ドリブル
        if dribble and not ready_for_shoot:
            Action.dribble()


class Action:  # 機構制御関数を格納するクラス

    # 射出準備
    def ready_for_shoot():
        global ready_for_shoot
        print("Ready for Shooting...")
        data[9] = 1
        data[11] = 1
        data[1] = -1 * roller_speed_shoot_ab
        data[2] = -1 * roller_speed_shoot_ab
        data[3] = roller_speed_shoot_cd
        data[4] = roller_speed_shoot_cd
        udp.send()  # UDPで送信
        ready_for_shoot = True
        print("Done.")

    # 射出シーケンス
    def shoot():
        global ready_for_shoot
        print("Shooting...")
        data[10] = 1
        udp.send()  # 　UDPで送信
        ready_for_shoot = False
        time.sleep(1.0)
        print("Ready for Retraction...")
        data[10] = -1
        udp.send()  # 　UDPで送信
        time.sleep(1.0)
        print("Retracting....")
        data[9] = -1
        data[11] = -1
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send()  # UDPで送信
        print("Done.")

    # ドリブル
    def dribble():
        print("Dribbling...")
        data[11] = 1
        data[1] = roller_speed_dribble_ab
        data[2] = roller_speed_dribble_ab
        data[3] = -1 * roller_speed_dribble_cd
        data[4] = -1 * roller_speed_dribble_cd
        udp.send()  # UDPで送信
        time.sleep(6.0)
        data[11] = -1
        data[1] = 0
        data[2] = 0
        data[3] = 0
        data[4] = 0
        udp.send()  # UDPで送信
        print("Done.")


class UDP:  # UDP通信のクラス
    def __init__(self):

        SrcIP = "192.168.8.196"  # 送信元IP
        SrcPort = 4000  # 送信元ポート番号
        self.SrcAddr = (SrcIP, SrcPort)  # アドレスをtupleに格納

        DstIP = "192.168.8.216"  # 宛先IP
        DstPort = 5000  # 宛先ポート番号
        self.DstAddr = (DstIP, DstPort)  # アドレスをtupleに格納

        self.udpClntSock = socket(AF_INET, SOCK_DGRAM)  # ソケット作成
        try:  # 送信元アドレスでバインド
            self.udpClntSock.bind(self.SrcAddr)
        except:  # 例外処理、バインドに失敗したときはオフラインモードで開始
            print("Cannot assign requested address.\nOFFLINE Mode started.")
            ONLINE_MODE = False

    def send(self):

        # print(data[1], data[2], data[3], data[4])
        """
        配列拡張に伴い一時的に無効化 
        # Duty比のリミッター、消すな! 
        for i in range(len(data)):
            if data[i] > duty_max:
                data[i] = duty_max
            elif data[i] < -duty_max:
                data[i] = -duty_max
        """
        str_data = ",".join(map(str, data[1:17]))  # パケットを作成、配列要素をstr型にキャストしてカンマ区切りで結合

        print(str_data)

        send_data = str_data.encode("utf-8")  # バイナリに変換

        if ONLINE_MODE:
            self.udpClntSock.sendto(send_data, self.DstAddr)  # 宛先アドレスに送信

        #機構制御なので初期化は無効化
        # data[1] = 0
        # data[2] = 0
        # data[3] = 0
        # data[4] = 0
        # data[5] = 0
        # ata[6] = 0
        # data[7] = 0
        # data[8] = 0


udp = UDP()


def main(args=None):
    rclpy.init(args=args)
    exec = SingleThreadedExecutor()

    listener = Listener()

    if GUI_PARAM_MODE:
        param_listener = Param_Listener()

    exec.add_node(listener)
    if GUI_PARAM_MODE:
        exec.add_node(param_listener)

    exec.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    listener.destroy_node()
    if GUI_PARAM_MODE:
        param_listener.destroy_node()
    exec.shutdown()


if __name__ == "__main__":
    main()
