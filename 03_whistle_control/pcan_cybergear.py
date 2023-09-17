import struct
import can
import logging

class CANMotorController:
    PARAMETERS = {
        "run_mode": {"index": 0x7005, "format": "u8"},
        "iq_ref": {"index": 0x7006, "format": "f"},
        "spd_ref": {"index": 0x700A, "format": "f"},
        "limit_torque": {"index": 0x700B, "format": "f"},
        "cur_kp": {"index": 0x7010, "format": "f"},
        "cur_ki": {"index": 0x7011, "format": "f"},
        "cur_filt_gain": {"index": 0x7014, "format": "f"},
        "loc_ref": {"index": 0x7016, "format": "f"},
        "limit_spd": {"index": 0x7017, "format": "f"},
        "limit_cur": {"index": 0x7018, "format": "f"},
    }
    TWO_BYTES_BITS = 16

    def __init__(self, bus, motor_id=127, main_can_id=254):
        self.bus = bus
        self.MOTOR_ID = motor_id
        self.MAIN_CAN_ID = main_can_id
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.KP_MIN, self.KP_MAX = 0.0, 500.0  # 0.0 ~ 500.0
        self.KD_MIN, self.KD_MAX = 0.0, 5.0  # 0.0 ~ 5.0

    # 通信类型
    class CmdModes:
        GET_DEVICE_ID = 0
        MOTOR_CONTROL = 1
        MOTOR_FEEDBACK = 2
        MOTOR_ENABLE = 3
        MOTOR_STOP = 4
        SET_MECHANICAL_ZERO = 6
        SET_MOTOR_CAN_ID = 7
        SINGLE_PARAM_READ = 17
        SINGLE_PARAM_WRITE = 18
        FAULT_FEEDBACK = 21

    def _float_to_uint(self, x, x_min, x_max, bits):
        span = x_max - x_min
        offset = x_min
        x = max(min(x, x_max), x_min)  # Clamp x to the range [x_min, x_max]
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def _uint_to_float(self, x, x_min, x_max, bits):
        span = (1 << bits) - 1
        offset = x_max - x_min
        x = max(min(x, span), 0)  # Clamp x to the range [0, span]
        return offset * x / span + x_min

    # 定义线性映射函数
    def _linear_mapping(
        self, value, value_min, value_max, target_min=0, target_max=65535
    ):
        return int(
            (value - value_min) / (value_max - value_min) * (target_max - target_min)
            + target_min
        )

    def format_data(self, data=[], format="f f", type="decode"):
        format_list = format.split()
        rdata = []
        if type == "decode":
            p = 0
            for f in format_list:
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                ba = bytearray()
                if len(s_f) == 2:
                    for i in range(s_f[0]):
                        ba.append(data[p])
                        p = p + 1
                    rdata.append(struct.unpack(s_f[1], ba)[0])
                else:
                    logging.info("unknown format in format_data(): " + f)
                    return []
            return rdata
        elif type == "encode" and len(format_list) == len(data):
            for i in range(len(format_list)):
                f = format_list[i]
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                if f != "f":
                    data[i] = int(data[i])
                if len(s_f) == 2:
                    bs = struct.pack(s_f[1], data[i])
                    for j in range(s_f[0]):
                        rdata.append(bs[j])
                else:
                    logging.info("unkown format in format_data(): " + f)
                    return []
            if len(rdata) < 4:
                for i in range(4 - len(rdata)):
                    rdata.append(0x00)
            return rdata

    # 定义打包data1函数
    def pack_to_8bytes(self, target_angle, target_velocity, Kp, Kd):
        # 对输入变量进行线性映射
        target_angle_mapped = self._linear_mapping(target_angle, self.P_MIN, self.P_MAX)
        target_velocity_mapped = self._linear_mapping(
            target_velocity, self.V_MIN, self.V_MAX
        )
        Kp_mapped = self._linear_mapping(Kp, self.KP_MIN, self.KP_MAX)
        Kd_mapped = self._linear_mapping(Kd, self.KD_MIN, self.KD_MAX)

        # 使用Python的struct库进行打包
        # 使用H表示无符号短整数（2字节），共需要8字节
        data1_bytes = struct.pack(
            "HHHH", target_angle_mapped, target_velocity_mapped, Kp_mapped, Kd_mapped
        )
        data1 = [b for b in data1_bytes]
        return data1

    def send_receive_can_message(self, cmd_mode, data2, data1, timeout=200):
        """
        Send a CAN message and receive a response.
        
        Parameters:
        - cmd_mode: Command mode (bit28~bit24)
        - data2: Data area 2 (bit23~bit8)
        - data1: Data bytes to send
        - timeout: Timeout for sending the message (default is 200ms)
        
        Returns:
        A tuple containing the received message data and the received message arbitration ID, if any.
        """
        # Calculate the arbitration ID
        arbitration_id = (cmd_mode << 24) | (data2 << 8) | self.MOTOR_ID
        message = can.Message(
            arbitration_id=arbitration_id, data=data1, is_extended_id=True
        )

        # Send the CAN message
        try:
            self.bus.send(message)
        except:
            logging.info("Failed to send the message.")
            return None, None

        # Output details of the sent message
        logging.debug(f"Sent message with ID {hex(arbitration_id)}, data: {data1}")

        # Receive a CAN message with a 1-second timeout
        received_msg = self.bus.recv(timeout=1)  # 1-second timeout for receiving
        if received_msg:
            return received_msg.data, received_msg.arbitration_id
        else:
            return None, None

    def parse_received_msg(self, data, arbitration_id):
        if data is not None:
            logging.debug(f"Received message with ID {hex(arbitration_id)}")
            pos = self._uint_to_float(
                (data[0] << 8) + data[1], self.P_MIN, self.P_MAX, self.TWO_BYTES_BITS
            )
            vel = self._uint_to_float(
                (data[2] << 8) + data[3], self.V_MIN, self.V_MAX, self.TWO_BYTES_BITS
            )
            logging.info(f"pos:{pos:.2f} rad, vel:{vel:.2f} rad/s")
            return pos, vel
        else:
            logging.info("No message received within the timeout period.")
            return None, None

    def clear_can_rx(self, timeout=10):
        """
        Clear the receiver buffer by reading all existing messages.
        
        Parameters:
        - timeout: Time in ms to wait for the clearing operation.
        """
        timeout_seconds = timeout / 1000.0  # Convert to seconds
        while True:
            received_msg = self.bus.recv(timeout=timeout_seconds)
            if received_msg is None:
                break
            logging.info(f"Cleared message with ID {hex(received_msg.arbitration_id)}")

    def _write_single_param(self, index, value, format="u32"):
        encoded_data = self.format_data(data=[value], format=format, type="encode")
        data1 = [b for b in struct.pack("<I", index)] + encoded_data

        self.clear_can_rx()  # 空CAN接收缓存，避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SINGLE_PARAM_WRITE,
            data2=self.MAIN_CAN_ID,
            data1=data1,
        )
        self.parse_received_msg(received_msg_data, received_msg_arbitration_id)
        return received_msg_data, received_msg_arbitration_id

    def write_single_param(self, param_name, value):
        param_info = self.PARAMETERS.get(param_name)
        if param_info is None:
            logging.info(f"Unknown parameter name: {param_name}")
            return

        index = param_info["index"]
        format = param_info["format"]

        self._write_single_param(index=index, value=value, format=format)

    def set_0_pos(self):
        self.clear_can_rx()  # 清空CAN接收缓存，避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SET_MECHANICAL_ZERO,
            data2=self.MAIN_CAN_ID,
            data1=[1],
        )

        self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def enable(self):
        self.clear_can_rx(0)  # 清空CAN接收缓存，避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.MOTOR_ENABLE, data2=self.MAIN_CAN_ID, data1=[]
        )
        self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def disable(self):
        self.clear_can_rx(0)  # 空CAN接收缓存，避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.MOTOR_STOP,
            data2=self.MAIN_CAN_ID,
            data1=[0, 0, 0, 0, 0, 0, 0, 0],
        )
        self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def set_motor_position_control(self, limit_spd, loc_ref):
        # 设置电机最大速度
        self.write_single_param(param_name="limit_spd", value=limit_spd)
        # 设置电机目标位置
        self.write_single_param(param_name="loc_ref", value=loc_ref)