import serial.tools.list_ports
import os
import subprocess


def get_device_details(port):
    """获取设备详细信息"""
    details = []

    # 获取设备权限
    try:
        permission_info = (
            subprocess.check_output(["ls", "-la", port.device]).decode("utf-8").strip()
        )
        details.append(f"权限: {permission_info}")
    except Exception as e:
        details.append(f"权限检查错误: {e}")

    # USB设备信息
    if port.vid is not None:
        details.append(f"VID:PID: {port.vid:04X}:{port.pid:04X}")

    # 硬件ID信息
    if port.hwid:
        details.append(f"硬件ID: {port.hwid}")

    # 设备路径
    if hasattr(port, "device_path"):
        details.append(f"设备路径: {port.device_path}")

    return details


def main():
    print("系统上所有串口设备:")
    ports = serial.tools.list_ports.comports()

    if not ports:
        print("未找到任何串口设备")
        return

    for i, port in enumerate(ports):
        print(f"\n{i+1}. {port.device}: {port.description}")

        details = get_device_details(port)
        for detail in details:
            print(f"   - {detail}")

    print("\n检查系统日志中的串口设备信息:")
    try:
        dmesg_output = subprocess.check_output(
            ["dmesg", "|", "grep", "tty"], shell=True
        ).decode("utf-8")
        print(dmesg_output)
    except subprocess.CalledProcessError:
        print("无法获取系统日志信息")


if __name__ == "__main__":
    main()
