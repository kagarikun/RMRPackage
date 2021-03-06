#!/usr/bin/env python

#*******************************************************************************
# Copyright 2021 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#*******************************************************************************

#*******************************************************************************
# This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
# For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
# To test this example, please follow the commands below.
#
# Open terminal #1
# $ roscore
#
# Open terminal #2
# $ rosrun dynamixel_sdk_examples read_write_node.py
#
# Open terminal #3 (run one of below commands at a time)
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
# $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
# $ rosservice call /get_position "id: 1"
#
# Author: Will Son
#******************************************************************************/

import os
from dynamixel_sdk import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
OPERATING_MODE=11
ADDR_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
GOAL_VELOCITY=104
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyAMA1' 
VELOCITY_CONTROL_MODE=1
'++++++++++++++++++++++++++++++++'    # Check++
#which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold
LEFT_WHEEL_ID =2
RIGHT_WHEEL_ID=1
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def set_goal_pos_callback(data):
    print("Set Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)

def get_present_pos(req):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    return dxl_present_position

import rclpy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.node import Node

from robot_interface.msg import RobotMove
from std_msgs.msg import Int32

class Dx_sub(Node):

    def __init__(self):
        super().__init__('position_subscriber')
        qos = QoSProfile(depth=10,reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
    	#qos.QoSHistoryPolicy=QoSHistoryPolicy.
        self.subscription = self.create_subscription(RobotMove,"robotMove",self.listener_callback,10)
        self.subscription  # prevent unused variable warning
        portHandler.openPort()
        if portHandler.setBaudRate(BAUDRATE):
            print("success")
        else:
            print("miss")

    def listener_callback(self, msg):
        #torque
        msg.left_wheel_speed=-msg.left_wheel_speed
        dxl_comm_result,tes=packetHandler.write1ByteTxRx(portHandler,LEFT_WHEEL_ID,ADDR_TORQUE_ENABLE,1)
        dxl_comm_result,tes=packetHandler.write1ByteTxRx(portHandler,RIGHT_WHEEL_ID,ADDR_TORQUE_ENABLE,1)
        #controllMode
        packetHandler.write1ByteTxRx(portHandler, LEFT_WHEEL_ID,OPERATING_MODE , VELOCITY_CONTROL_MODE)
        packetHandler.write1ByteTxRx(portHandler, RIGHT_WHEEL_ID,OPERATING_MODE , VELOCITY_CONTROL_MODE)
        #Speed
        packetHandler.write4ByteTxRx(portHandler, LEFT_WHEEL_ID, GOAL_VELOCITY, msg.left_wheel_speed)
        packetHandler.write4ByteTxRx(portHandler, RIGHT_WHEEL_ID, GOAL_VELOCITY, msg.right_wheel_speed)
        print(msg.left_wheel_speed)
        print(msg.right_wheel_speed)
        print("end--------------")


def main(args=None):
    rclpy.init(args=args)

    dx_sub = Dx_sub()

    rclpy.spin(dx_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dx_sub.destroy_node()
    packetHandler.write1ByteTxRx(portHandler,1,ADDR_TORQUE_ENABLE,0)
    rclpy.shutdown()
    portHandler.closePort()


if __name__ == '__main__':
    main()
