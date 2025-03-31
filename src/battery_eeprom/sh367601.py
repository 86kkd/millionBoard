import serial
import time
import struct
import argparse

class SH367601X:
    """SH3676010/3676014/3676016系列锂电池保护芯片通信类"""
    
    def __init__(self, port, baudrate=9600, timeout=1):
        """初始化串口连接
        
        Args:
            port: 串口设备名称
            baudrate: 波特率，芯片固定为9600
            timeout: 超时时间(秒)
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        self.DEVICE_ADDR = 0x1C  # 芯片地址固定为0x1C
        time.sleep(0.5)  # 等待连接稳定
        
    def close(self):
        """关闭串口连接"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            
    def calculate_crc8(self, data):
        """计算CRC8校验值
        
        Args:
            data: 字节列表或字节数组
            
        Returns:
            计算得到的CRC8校验值
        """
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
    
    def send_command(self, cmd, data=None):
        """发送命令并接收响应
        
        Args:
            cmd: 命令字节
            data: 要发送的数据字节列表(可选)
            
        Returns:
            接收到的响应数据(不包含CRC)
        """
        # 构建发送帧
        frame = [self.DEVICE_ADDR, cmd]
        
        if data:
            frame.extend(data)
            
        # 计算并添加CRC8校验
        crc = self.calculate_crc8(frame)
        frame.append(crc)
        
        # 清空接收缓冲区
        self.ser.reset_input_buffer()
        
        # 发送命令
        self.ser.write(bytes(frame))
        
        # 等待响应
        time.sleep(0.05)
        
        # 读取响应
        response = self.ser.read(100)  # 读取足够多的字节
        
        if len(response) < 3:  # 至少应该有地址、命令和CRC
            print(f"接收响应失败，接收到 {len(response)} 字节")
            return None
            
        # 验证CRC
        received_crc = response[-1]
        calculated_crc = self.calculate_crc8(response[:-1])
        
        if received_crc != calculated_crc:
            print(f"CRC校验失败: 接收={received_crc:02X}, 计算={calculated_crc:02X}")
            return None
            
        # 返回响应数据(不包含地址、命令和CRC)
        return response[2:-1]
    
    def read_register(self, reg_addr):
        """读取寄存器值
        
        Args:
            reg_addr: 寄存器地址
            
        Returns:
            寄存器值(字节列表)或None(失败时)
        """
        # 读寄存器命令为0x01
        cmd = 0x01
        data = [reg_addr]
        
        response = self.send_command(cmd, data)
        return response
    
    def write_register(self, reg_addr, reg_value):
        """写入寄存器值
        
        Args:
            reg_addr: 寄存器地址
            reg_value: 要写入的值(字节列表)
            
        Returns:
            True(成功)或False(失败)
        """
        # 写寄存器命令为0x02
        cmd = 0x02
        data = [reg_addr]
        data.extend(reg_value)
        
        response = self.send_command(cmd, data)
        return response is not None
    
    def read_cell_voltages(self):
        """读取所有电芯电压
        
        Returns:
            电芯电压列表(单位: mV)
        """
        voltages = []
        
        # 根据数据手册，电芯电压寄存器从0x10开始
        for i in range(16):
            reg_addr = 0x10 + i
            response = self.read_register(reg_addr)
            
            if response and len(response) >= 2:
                # 根据数据手册第16页的电芯电压计算公式：VCELL = CELL * 5 / 32
                voltage = int.from_bytes(response, byteorder='big', signed=True) * 5 / 32
                voltages.append(voltage)
            else:
                voltages.append(None)
                
        return voltages
    
    def read_current(self):
        """读取电流值
        
        Returns:
            电流值(单位: mA)，正值表示放电，负值表示充电
        """
        # 电流寄存器地址为0x20
        response = self.read_register(0x20)
        
        if response and len(response) >= 2:
            # 根据数据手册第16页的电流计算公式：Current = CUR * 90 / 26214.4 / RSENSE
            # 这里假设RSENSE = 1mΩ
            current_raw = int.from_bytes(response, byteorder='big', signed=True)
            current = current_raw * 90 / 26214.4 / 0.001  # 假设采样电阻为1mΩ
            return current
        else:
            return None
    
    def read_temperatures(self):
        """读取温度传感器值
        
        Returns:
            温度传感器电阻值列表(单位: kΩ)
        """
        temperatures = []
        
        # 温度寄存器地址从0x21开始
        for i in range(3):
            reg_addr = 0x21 + i
            response = self.read_register(reg_addr)
            
            if response and len(response) >= 2:
                # 根据数据手册第16页的温度计算公式：RT = 10 * TEMP / (32768 - TEMP)
                temp_raw = int.from_bytes(response, byteorder='big', signed=True)
                if temp_raw < 32768:
                    resistance = 10 * temp_raw / (32768 - temp_raw)
                    temperatures.append(resistance)
                else:
                    temperatures.append(None)
            else:
                temperatures.append(None)
                
        return temperatures
    
    def read_status(self):
        """读取芯片状态寄存器
        
        Returns:
            状态寄存器值的字典解释
        """
        # 状态寄存器地址从0x01开始
        status = {}
        
        # 读取状态寄存器1
        response = self.read_register(0x01)
        if response and len(response) >= 1:
            status_byte = response[0]
            status['OV'] = bool(status_byte & 0x01)  # 过充电保护状态
            status['UV'] = bool(status_byte & 0x02)  # 过放电保护状态
            status['OCD1'] = bool(status_byte & 0x04)  # 放电过流1保护状态
            status['OCD2'] = bool(status_byte & 0x08)  # 放电过流2保护状态
            status['SC'] = bool(status_byte & 0x10)  # 短路保护状态
            status['OCC'] = bool(status_byte & 0x20)  # 充电过流保护状态
            status['L0V'] = bool(status_byte & 0x40)  # 低电压禁止充电状态
            status['OW'] = bool(status_byte & 0x80)  # 断线保护状态
        
        # 读取状态寄存器2
        response = self.read_register(0x02)
        if response and len(response) >= 1:
            status_byte = response[0]
            status['UTC'] = bool(status_byte & 0x01)  # 充电低温保护状态
            status['OTC'] = bool(status_byte & 0x02)  # 充电高温保护状态
            status['UTD'] = bool(status_byte & 0x04)  # 放电低温保护状态
            status['OTD'] = bool(status_byte & 0x08)  # 放电高温保护状态
            status['OTI'] = bool(status_byte & 0x10)  # 芯片内部高温保护状态
            status['CTLD'] = bool(status_byte & 0x20)  # CTLD管脚控制状态
            status['PD'] = bool(status_byte & 0x40)  # Powerdown模式状态
            status['BAL'] = bool(status_byte & 0x80)  # 均衡状态
            
        # 读取状态寄存器3
        response = self.read_register(0x03)
        if response and len(response) >= 1:
            status_byte = response[0]
            status['CHGING'] = bool(status_byte & 0x01)  # 充电状态
            status['DSGING'] = bool(status_byte & 0x02)  # 放电状态
            
        return status
    
    def read_eeprom(self, addr):
        """读取EEPROM内容
        
        Args:
            addr: EEPROM地址
            
        Returns:
            EEPROM内容(字节)
        """
        # EEPROM读取命令为0x03
        cmd = 0x03
        data = [addr]
        
        response = self.send_command(cmd, data)
        if response:
            return response[0]
        return None
    
    def write_eeprom(self, addr, value):
        """写入EEPROM内容
        
        Args:
            addr: EEPROM地址
            value: 要写入的值(字节)
            
        Returns:
            True(成功)或False(失败)
        """
        # EEPROM写入命令为0x04
        cmd = 0x04
        data = [addr, value]
        
        response = self.send_command(cmd, data)
        return response is not None
    
    def eeprom_write_enable(self):
        """使能EEPROM写入
        
        Returns:
            True(成功)或False(失败)
        """
        # EEPROM写使能命令为0x05
        cmd = 0x05
        
        response = self.send_command(cmd)
        return response is not None
    
    def eeprom_write_disable(self):
        """禁止EEPROM写入
        
        Returns:
            True(成功)或False(失败)
        """
        # EEPROM写禁止命令为0x06
        cmd = 0x06
        
        response = self.send_command(cmd)
        return response is not None
    
    def configure_cell_count(self, cell_count):
        """配置电池串数
        
        Args:
            cell_count: 电池串数(6-16)
            
        Returns:
            True(成功)或False(失败)
        """
        if cell_count < 6 or cell_count > 16:
            print("错误：电池串数必须在6-16之间")
            return False
            
        # 读取当前配置
        current_config = self.read_eeprom(0x04)
        if current_config is None:
            return False
            
        # 计算串数配置值
        cn_value = cell_count - 6
        
        # 修改配置(保留高4位，修改低4位)
        new_config = (current_config & 0xF0) | cn_value
        
        # 写入EEPROM
        self.eeprom_write_enable()
        result = self.write_eeprom(0x04, new_config)
        self.eeprom_write_disable()
        
        return result
    
    def configure_protection_voltage(self, ov_voltage=4200, uv_voltage=2500):
        """配置过充电和过放电保护电压
        
        Args:
            ov_voltage: 过充电保护电压(mV)，范围2500-4500
            uv_voltage: 过放电保护电压(mV)，范围1000-3500
            
        Returns:
            True(成功)或False(失败)
        """
        # 检查参数范围
        if ov_voltage < 2500 or ov_voltage > 4500:
            print("错误：过充电保护电压必须在2500-4500mV之间")
            return False
            
        if uv_voltage < 1000 or uv_voltage > 3500:
            print("错误：过放电保护电压必须在1000-3500mV之间")
            return False
            
        # 计算寄存器值
        # 根据数据手册，过充电保护电压步进为5mV
        ov_reg_value = (ov_voltage - 2500) // 5
        
        # 过放电保护电压步进为10mV
        uv_reg_value = (uv_voltage - 1000) // 10
        
        # 写入EEPROM
        self.eeprom_write_enable()
        result1 = self.write_eeprom(0x05, ov_reg_value)
        result2 = self.write_eeprom(0x07, uv_reg_value)
        self.eeprom_write_disable()
        
        return result1 and result2
    
    def software_reset(self):
        """软件复位
        
        Returns:
            True(成功)或False(失败)
        """
        # 软件复位命令为0x07
        cmd = 0x07
        
        response = self.send_command(cmd)
        return response is not None


def main():
    """主函数，用于命令行操作"""
    parser = argparse.ArgumentParser(description='SH367601X系列芯片UART通信工具')
    parser.add_argument('port', help='串口设备名称')
    parser.add_argument('--read-voltages', action='store_true', help='读取电芯电压')
    parser.add_argument('--read-current', action='store_true', help='读取电流')
    parser.add_argument('--read-temps', action='store_true', help='读取温度')
    parser.add_argument('--read-status', action='store_true', help='读取状态')
    parser.add_argument('--cell-count', type=int, help='设置电池串数(6-16)')
    parser.add_argument('--ov', type=int, help='设置过充电保护电压(mV)')
    parser.add_argument('--uv', type=int, help='设置过放电保护电压(mV)')
    parser.add_argument('--reset', action='store_true', help='执行软件复位')
    
    args = parser.parse_args()
    
    try:
        chip = SH367601X(args.port)
        
        if args.read_voltages:
            voltages = chip.read_cell_voltages()
            print("电芯电压(mV):")
            for i, v in enumerate(voltages):
                if v is not None:
                    print(f"CELL{i+1}: {v:.1f} mV")
                    
        if args.read_current:
            current = chip.read_current()
            if current is not None:
                print(f"电流: {current:.2f} mA")
                
        if args.read_temps:
            temps = chip.read_temperatures()
            print("温度传感器电阻(kΩ):")
            for i, t in enumerate(temps):
                if t is not None:
                    print(f"TS{i+1}: {t:.2f} kΩ")
                    
        if args.read_status:
            status = chip.read_status()
            print("芯片状态:")
            for key, value in status.items():
                print(f"{key}: {'活跃' if value else '非活跃'}")
                
        if args.cell_count:
            if chip.configure_cell_count(args.cell_count):
                print(f"电池串数设置为 {args.cell_count} 成功")
            else:
                print("电池串数设置失败")
                
        if args.ov or args.uv:
            ov = args.ov if args.ov else 4200
            uv = args.uv if args.uv else 2500
            if chip.configure_protection_voltage(ov, uv):
                print(f"保护电压设置成功: OV={ov}mV, UV={uv}mV")
            else:
                print("保护电压设置失败")
                
        if args.reset:
            if chip.software_reset():
                print("软件复位执行成功")
            else:
                print("软件复位执行失败")
                
    except Exception as e:
        print(f"错误: {e}")
        
    finally:
        if 'chip' in locals():
            chip.close()


if __name__ == "__main__":
    main()

