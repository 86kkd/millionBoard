import serial
import time


class SH367601X:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout,
        )
        self.DEVICE_ADDR = 0x1C  # 假设设备地址为0x1C，可能需要调整

        # 清空缓冲区
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        # 等待芯片初始化完成
        time.sleep(0.1)

        # 发送唤醒序列
        self._send_wakeup()

    def _send_wakeup(self):
        """发送唤醒序列，尝试激活UART通信模块"""
        # 方法1: 发送0x00字节
        self.ser.write(bytes([0x00]))
        time.sleep(0.01)
        self.ser.reset_input_buffer()

        # 方法2: 触发RXD下降沿
        # 如果支持DTR/RTS控制，可以尝试以下代码
        if hasattr(self.ser, "dtr"):
            self.ser.dtr = True
            time.sleep(0.001)
            self.ser.dtr = False
            time.sleep(0.01)

    def calculate_crc8(self, data):
        """计算CRC8校验值"""
        crc = 0
        polynomial = 0x07  # CRC8多项式 x^8 + x^2 + x + 1

        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ polynomial
                else:
                    crc = crc << 1
                crc &= 0xFF

        return crc

    def send_command(self, cmd, data=None, retries=3):
        """发送命令并接收响应，支持多次重试"""
        success = False
        response = None

        for attempt in range(retries):
            # 构建发送帧
            frame = [self.DEVICE_ADDR, cmd]

            if data:
                frame.extend(data)

            # 计算并添加CRC8校验
            crc = self.calculate_crc8(frame)
            frame.append(crc)

            # 清空接收缓冲区
            self.ser.reset_input_buffer()

            # 打印发送的数据(调试用)
            print(f"发送: {[hex(b) for b in frame]}")

            # 发送命令
            self.ser.write(bytes(frame))

            # 等待响应
            time.sleep(0.05)

            # 读取响应
            if self.ser.in_waiting > 0:
                response = self.ser.read(self.ser.in_waiting)
                print(f"接收: {[hex(b) for b in response]}")
                success = True
                break
            else:
                print(f"尝试 {attempt+1}/{retries}: 无响应")
                # 重试前再次发送唤醒序列
                self._send_wakeup()
                time.sleep(0.1 * (attempt + 1))  # 递增等待时间

        return response if success else None

    def read_register(self, reg_addr):
        """读取寄存器值"""
        # 尝试不同的读寄存器命令
        for cmd in [0x01, 0x03, 0x10]:  # 常见的读命令
            print(f"尝试使用命令 {hex(cmd)} 读取寄存器 {hex(reg_addr)}")
            response = self.send_command(cmd, [reg_addr])
            if response:
                return response
        return None

    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()


def test_communication():
    """测试与芯片的通信"""
    port = "/dev/ttyACM0"  # 根据实际情况修改

    try:
        print(f"尝试连接到 {port}...")
        chip = SH367601X(port)

        # 测试读取设备ID (通常在寄存器0x00)
        print("尝试读取设备ID...")
        response = chip.read_register(0x00)
        if response:
            print(f"成功读取设备ID: {[hex(b) for b in response]}")
        else:
            print("读取设备ID失败")

        # 尝试读取状态寄存器
        print("尝试读取状态寄存器...")
        for reg in [0x01, 0x02, 0x03]:
            response = chip.read_register(reg)
            if response:
                print(f"成功读取寄存器 {hex(reg)}: {[hex(b) for b in response]}")
            else:
                print(f"读取寄存器 {hex(reg)} 失败")

    except Exception as e:
        print(f"通信测试出错: {e}")

    finally:
        if "chip" in locals():
            chip.close()
            print("串口已关闭")


if __name__ == "__main__":
    test_communication()
