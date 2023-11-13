import os
import sys
import csv
# 添加pcan_cybergear库的路径
sys.path.append(os.path.join("..", "cybergear"))

from pcan_cybergear import CANMotorController
import can
import logging
import time
# Initialize logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class RoboReplay:
    def __init__(self) -> None:
        # Connect to the CAN bus with 1 Mbit/s bitrate
        self.bus = can.interface.Bus(bustype="pcan", channel="PCAN_USBBUS1", bitrate=1000000)
        self.motor1 = CANMotorController(self.bus, motor_id=101, main_can_id=254)
        self.motor2 = CANMotorController(self.bus, motor_id=102, main_can_id=254)
        self.motor3 = CANMotorController(self.bus, motor_id=103, main_can_id=254)
        self.motor4 = CANMotorController(self.bus, motor_id=104, main_can_id=254)

        self.motors = [self.motor1, self.motor2, self.motor3, self.motor4]
        
        for motor in self.motors:
            motor.disable()
            motor.set_0_pos()
            motor.set_run_mode(motor.RunModes.POSITION_MODE) # 位置模式
            motor.write_single_param("loc_ref", value=0) # 目标位置

        self.motor1.write_single_param("limit_spd", value=1) # 最大速度 rad/s
        self.motor2.write_single_param("limit_spd", value=1) # 最大速度 rad/s
        self.motor3.write_single_param("limit_spd", value=1) # 最大速度 rad/s
        self.motor4.write_single_param("limit_spd", value=1) # 最大速度 rad/s
        
        # 从CSV文件加载点位信息
    def load_positions_from_csv(self, filename):
        positions = []
        with open(filename, mode='r') as file:
            csv_reader = csv.reader(file)
            next(csv_reader)  # skip header
            for row in csv_reader:
                positions.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
        logging.info(f"length of positions: {len(positions)}")
        return positions
    
    def enable(self):
        for motor in self.motors:
            motor.write_single_param("loc_ref", value=0)
            motor.enable()
    def disable(self):
        for motor in self.motors:
            motor.disable()
    def replay(self, filename):
        loaded_positions = self.load_positions_from_csv(filename)
        for position in loaded_positions:
            logging.info(position)
            for i, motor in enumerate(self.motors):
                motor.write_single_param("loc_ref", value=position[i])
            time.sleep(1)  # 等待电机移动到目标位置
        for motor in self.motors:
            motor.write_single_param("loc_ref", value=0)