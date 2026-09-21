#!/usr/bin/env python3

################################################################################
# Copyright (c) 2024,D-Robotics.
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
################################################################################

import sys
import signal
import os
import time

# 导入python串口库
import serial
import serial.tools.list_ports

def signal_handler(signal, frame):
    sys.exit(0)

def serialTest():
    print("List of enabled UART:")
    os.system('ls /dev/tty[a-zA-Z]*')
    uart_dev= input("Please enter the name of the serial device to be tested:")

    baudrate = input("Please enter the baud rate(9600,19200,38400,57600,115200,921600):")
    try:
        ser = serial.Serial(uart_dev, int(baudrate), timeout=1) # 1s timeout
    except Exception as e:
        print("open serial failed!\n")

    print(ser)

    print("Starting demo now! Press CTRL+C to exit")

    while True:
        test_data = "AA55"
        write_num = ser.write(test_data.encode('UTF-8'))
        print("Send: ", test_data)

        received_data = ser.read(write_num).decode('UTF-8')
        print("Recv: ", received_data)

        time.sleep(1)

    ser.close()
    return 0


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    if serialTest() != 0:
        print("Serial test failed!")
    else:
        print("Serial test success!")
