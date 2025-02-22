#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#

import sys
import os
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from rclpy.executors import MultiThreadedExecutor
import numpy as np

sys.path.append("..")
from scservo_sdk import *                      # Uses FTServo SDK library

class AngleSubscriber(Node):
    def __init__(self):
        super().__init__('angle_subscriber')
        
        # 创建订阅者，订阅名为 'angles' 的话题
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'angles',
            self.listener_callback,
            10)
        self.subscription  # 让订阅对象不被垃圾回收

        self.latest_angles = [0,0,0,0,0,0,0,0]
        self.lock = threading.Lock()

    def listener_callback(self, msg):
        with self.lock:
            if(np.nan in msg.data):
                return
            self.latest_angles = msg.data  # 更新最近的数据

# 拇指侧摆关节 ID 7
# 拇指第二关节 ID 6
# 拇指质监关节 ID 5

# 舵机数量
SERVO_NUM = 3

class ServoControlTypedef(sms_sts):
    def __init__(self, portHandler, EncoderData=None):
        super().__init__(portHandler)
        
        self.EncoderData = EncoderData
        self.running = True
        
        # 舵机初始角度 0-4095
        self.InitServoPos = [0,0,0,0,0,4090,332,3562]
        # 编码器零位角度 0-360
        self.InitEncoderAngle = [283.7109375, 53.4375, 188.98681640625, 70.64208984375, 138.14208984375, 166.728515625, 126.01318359375, 138.6474609375]
        # 舵机目标位置 0-4095
        self.TargetServoPos = [0,0,0,0,0,0,0,0]
        # 舵机最大转角
        self.MaxServoPos = [0,0,0,0,0,5487,1676,4715]
        # 舵机最小转角
        self.MinServoPos = [0,0,0,0,0,2912,-3519,2645]
        
        # 舵机原始状态 位置、速度、电流（扭矩）整形
        self.int_NowServoPos = [0,0,0,0,0,0,0,0]
        self.int_NowServoVel = [0,0,0,0,0,0,0,0] 
        self.int_NowServoCur = [0,0,0,0,0,0,0,0] #[0,1000]
        
        #舵机状态 位置、速度、电流（扭矩）浮点
        self.NowServoPos = [0,0,0,0,0,0,0,0]
        self.NowServoVel = [0,0,0,0,0,0,0,0]
        self.NowServoCur = [0,0,0,0,0,0,0,0]
        
        # 编码器角度、圈数
        self.EncoderNowAngle = [0,0,0,0,0,0,0,0]
        self.EncoderLastAngle = [283.7109375, 53.4375, 188.98681640625, 70.64208984375, 138.14208984375, 166.728515625, 126.01318359375, 138.6474609375]
        self.EncoderCircle = [0,0,0,0,0,0,0,0]
        
        # 力控参数
        self.Kd = 1.0
        self.Bd = 0.0
        self.Md = 0.0
        
        # 力控开关
        self.SOFT_CONTROL = 0
        
        # 力矩、目标力矩
        self.TorqueErr = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        self.TargetTorque = [0,0,0,0,0,0,0,0]
        
        # 与外骨骼旋转方向
        self.where = [1,1,1,1,1,1,1,1]
    
    def ServoInit(self):
        for i in range(5,8):
            # 设置响应等级为0 只响应Ping和Read
            self.ChangeLevelResponse(i+1, 0)
            # 设置最大相电流为最大值1023mA 即1.023A
            self.ProtectCurrent(i+1, 1023)
            # 设置多圈模式(最大角度上限设置为0)
            self.MaxAngleSet(i+1, 0)
            
    def DataHandler(self, EncoderData, ID):
        self.EncoderNowAngle = EncoderData.latest_angles
        
        if(self.EncoderNowAngle[ID] - self.EncoderLastAngle[ID] > 180):
            self.EncoderCircle[ID] = self.EncoderCircle[ID] - 1
        elif(self.EncoderNowAngle[ID] - self.EncoderLastAngle[ID] < -180):
            self.EncoderCircle[ID] = self.EncoderCircle[ID] + 1
            
        self.EncoderLastAngle[ID] = self.EncoderNowAngle[ID]
        
        # 编码器过零点处理
        # for i in range(5,8):
        #     if(self.EncoderNowAngle[i] - self.EncoderLastAngle[i] > 180):
        #         self.EncoderCircle[i] = self.EncoderCircle[i] - 1
        #     elif(self.EncoderNowAngle[i] - self.EncoderLastAngle[i] < -180):
        #         self.EncoderCircle[i] = self.EncoderCircle[i] + 1
                
        #     self.EncoderLastAngle[i] = self.EncoderNowAngle[i]
            
    def GetServoStates(self, ID):
        self.int_NowServoPos[ID], scs_comm_result, scs_error  = self.ReadPos(ID+1)
        
        # for i in range(5,8):
        #     self.int_NowServoCur[i], scs_comm_result, scs_error = self.ReadCur(i+1) #[2048->1024 0->1024]
        #     # self.int_NowServoPos[i], self.int_NowServoVel[i], scs_comm_result, scs_error  = self.ReadPosSpeed(i+1)
        #     if(self.int_NowServoCur[i] > 1024):
        #         self.int_NowServoCur[i] = 1024 - self.int_NowServoCur[i]
            
        #     self.NowServoCur[i] = self.int_NowServoCur[i] / 10 # <0 逆时针 >0 顺时针 范围[-102.4,102.4]
        # print(f'ServoCur = {self.NowServoCur}')
    
    def ServoControl(self, ID):
        
        # Step 0 数据滤波
        self.EncoderNowAngle[ID] = self.EncoderNowAngle[ID] * 0.1 + self.EncoderLastAngle[ID] * 0.9
        
        # Step 1 转化为舵机对应整形角度值
        temp = (self.EncoderNowAngle[ID] + self.EncoderCircle[ID] * 360 - self.InitEncoderAngle[ID]) / 0.088
        # if()
        self.TargetServoPos[ID] = self.InitServoPos[ID] + self.where[ID] * int(temp)

        # 柔顺控制部分
        # if(self.SOFT_CONTROL):
            # self.TorqueErr[i] = self.NowServoCur[i] - self.TargetTorque
            # deltax = self.TorqueErr[i] / self.Kd
            # self.TargetServoPos[i] = self.TargetServoPos[i] + int(deltax)
            # # print("deltax%d:%f", i+1, deltax)
        
        # Step 2 根据舵机旋转最大和最小角度进行限幅
        if(self.TargetServoPos[ID] > self.MaxServoPos[ID]):
            self.TargetServoPos[ID] = self.MaxServoPos[ID]
        elif(self.TargetServoPos[ID] < self.MinServoPos[ID]):
            self.TargetServoPos[ID] = self.MinServoPos[ID]
            
        if(self.SOFT_CONTROL):
            self.TargetTorque[ID] = self.Kd * abs(self.TargetServoPos[ID] - self.int_NowServoPos[ID])
            if(self.TargetTorque[ID] < 16):
                self.TargetTorque[ID] = 16
            elif(self.TargetTorque[ID] > 1000):
                self.TargetTorque[ID] = 1000
            self.SetMaxTorque(ID+1, int(self.TargetTorque[ID]))
        
        if(self.TargetServoPos[ID] < 0):
            self.TargetServoPos[ID] = -1 * self.TargetServoPos[ID]
            self.TargetServoPos[ID] = (self.TargetServoPos[ID] | (1 << 15))
        
        # Step 3 发送控制指令
        self.WritePosEx(ID+1, self.TargetServoPos[ID], 32766, 0)
        # print(f'TargetTorque = {self.TargetTorque}')
        print(f'SetPos = {self.TargetServoPos}')
        
        
        # for i in range(5,8):
        #     # Step 0 数据滤波
        #     self.EncoderNowAngle[i] = self.EncoderNowAngle[i] * 0.05 + self.EncoderLastAngle[i] * 0.95
            
        #     # Step 1 转化为舵机对应整形角度值
        #     temp = (self.EncoderNowAngle[i] + self.EncoderCircle[i] * 360 - self.InitEncoderAngle[i]) / 0.088
        #     # if()
        #     self.TargetServoPos[i] = self.InitServoPos[i] + self.where[i] * int(temp)

        #     # 柔顺控制部分
        #     # if(self.SOFT_CONTROL):
        #         # self.TorqueErr[i] = self.NowServoCur[i] - self.TargetTorque
        #         # deltax = self.TorqueErr[i] / self.Kd
        #         # self.TargetServoPos[i] = self.TargetServoPos[i] + int(deltax)
        #         # # print("deltax%d:%f", i+1, deltax)
            
        #     # Step 2 根据舵机旋转最大和最小角度进行限幅
        #     if(self.TargetServoPos[i] > self.MaxServoPos[i]):
        #         self.TargetServoPos[i] = self.MaxServoPos[i]
        #     elif(self.TargetServoPos[i] < self.MinServoPos[i]):
        #         self.TargetServoPos[i] = self.MinServoPos[i]
                
        #     if(self.SOFT_CONTROL):
        #         self.TargetTorque[i] = self.Kd * abs(self.TargetServoPos[i] - self.int_NowServoPos[i])
        #         if(self.TargetTorque[i] < 16):
        #             self.TargetTorque[i] = 16
        #         elif(self.TargetTorque[i] > 1000):
        #             self.TargetTorque[i] = 1000
        #         self.SetMaxTorque(i+1, int(self.TargetTorque[i]))
            
        #     if(self.TargetServoPos[i] < 0):
        #         self.TargetServoPos[i] = -1 * self.TargetServoPos[i]
        #         self.TargetServoPos[i] = (self.TargetServoPos[i] | (1 << 15))
            
        #     # Step 3 发送控制指令
        #     self.WritePosEx(i+1, self.TargetServoPos[i], 15000, 255)
        # print(f'SetPos = {self.TargetServoPos}')
        
    def control_loop(self, ID):
        while self.running:
            # 执行舵机控制逻辑
            self.DataHandler(EncoderData=self.EncoderData, ID=ID)
            self.GetServoStates(ID=ID)
            self.ServoControl(ID=ID)
            
            time.sleep(0.01)  # 控制频率约1KHz
        

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler('/dev/ttyUSB0')# ex) sWindows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

# Initialize PacketHandler instance
# Get methods and members of Protocol
# packetHandler = sms_sts(portHandler)
    
# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    quit()

# Set port baudrate 1000000
if portHandler.setBaudRate(1000000):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    quit()

def main(args = None):
    rclpy.init(args=args)
    # EncoderData = AngleSubscriber()
    # ServoControlData = ServoControlTypedef(portHandler=portHandler)
    # # 初始化舵机参数
    # ServoControlData.ServoInit()
    
    try:
        # 创建ROS2节点
        EncoderData = AngleSubscriber()
        # 创建舵机控制器
        servo_controller = ServoControlTypedef(portHandler, EncoderData)
        servo_controller.ServoInit()
        servo_controller.running = True  # 控制循环运行标志

        # 创建多线程执行器
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(EncoderData)

        # 启动ROS2线程
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()

        # 启动舵机控制线程
        control5_thread = threading.Thread(
            target=servo_controller.control_loop, 
            kwargs={'ID': 5},  # 或 args=(5,)
            daemon=True
        )
        control5_thread.start()
        time.sleep(0.001)
        control6_thread = threading.Thread(
            target=servo_controller.control_loop, 
            kwargs={'ID': 6},  # 或 args=(5,)
            daemon=True
        )
        control6_thread.start()
        time.sleep(0.001)
        control7_thread = threading.Thread(
            target=servo_controller.control_loop, 
            kwargs={'ID': 7},  # 或 args=(5,)
            daemon=True                                   
        )
        control7_thread.start()

        # 主线程监控
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # 清理资源
        servo_controller.running = False
        control5_thread.join()
        control6_thread.join()
        control7_thread.join()
        executor.shutdown()
        EncoderData.destroy_node()
        rclpy.shutdown()
        portHandler.closePort()
    
    
    # try:
    #     while rclpy.ok():
    #         # 获取编码器角度
    #         rclpy.spin_once(EncoderData)
    #         # 更新角度 进行数据处理
    #         ServoControlData.DataHandler(EncoderData=EncoderData)
    #         # 读取舵机信息
    #         ServoControlData.GetServoStates()
    #         # 控制数据发送
    #         ServoControlData.ServoControl()
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     EncoderData.destroy_node()
    #     rclpy.shutdown()

if __name__ =="__main__":
    main()


# Close port
portHandler.closePort()
