import serial
import time
import argparse
import binascii


def main():
    parser = argparse.ArgumentParser(description="串口监视器")
    parser.add_argument("port", help="串口设备名称")
    parser.add_argument("--baudrate", type=int, default=9600, help="波特率(默认9600)")
    parser.add_argument("--timeout", type=float, default=0.1, help="读取超时时间(秒)")
    parser.add_argument("--duration", type=int, default=30, help="监视持续时间(秒)")

    args = parser.parse_args()

    try:
        print(f"尝试连接串口: {args.port}, 波特率: {args.baudrate}")

        # 打开串口
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baudrate,
            bytesize=8,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=args.timeout,
        )

        print(f"串口连接成功: {ser.name}")
        print(f"开始监视串口通信，持续 {args.duration} 秒...")
        print("按 Ctrl+C 终止")

        start_time = time.time()

        # 发送测试数据
        test_data = bytes([0x1C, 0x01, 0x10, 0x3D])  # SH367601X读取电芯电压命令
        print(f"\n发送测试数据: {' '.join([f'{b:02X}' for b in test_data])}")
        ser.write(test_data)

        # 持续监视
        while time.time() - start_time < args.duration:
            # 读取数据
            data = ser.read(100)
            if data:
                print(
                    f"接收: {' '.join([f'{b:02X}' for b in data])} | ASCII: {data.decode(errors='replace')}"
                )

            # 每2秒发送一次测试命令
            if int((time.time() - start_time) % 2) == 0:
                print(f"发送: {' '.join([f'{b:02X}' for b in test_data])}")
                ser.write(test_data)
                time.sleep(0.1)  # 短暂延迟以避免连续发送

        # 关闭串口
        ser.close()
        print("\n监视结束，串口已关闭")

    except KeyboardInterrupt:
        print("\n用户中断，停止监视")
        if "ser" in locals() and ser.is_open:
            ser.close()
            print("串口已关闭")
    except Exception as e:
        print(f"错误: {e}")
        if "ser" in locals() and ser.is_open:
            ser.close()


if __name__ == "__main__":
    main()
