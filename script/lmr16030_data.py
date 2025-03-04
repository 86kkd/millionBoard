import math
import numpy as np
import matplotlib.pyplot as plt


def calculate_rt(fsw_khz):
    """计算RT电阻值(欧姆)，基于LMR16030/LMR16020系列芯片"""
    fsw_hz = fsw_khz * 1000
    rt_ohm = 42904 / (fsw_hz**1.088)
    return rt_ohm


def calculate_feedback_resistors(vout, vref=0.75, rfbb=10000):
    """计算反馈电阻RFBT和RFBB"""
    rfbt = rfbb * (vout / vref - 1)
    return rfbt, rfbb


def calculate_inductor(vin, vout, iout_max, fsw_khz, ripple_current_ratio=0.3):
    """计算电感值"""
    fsw = fsw_khz * 1000
    # 计算占空比
    duty_cycle = vout / vin
    # 计算电感值 (单位: 亨利)
    ripple_current = iout_max * ripple_current_ratio
    l_value = (vin - vout) * duty_cycle / (ripple_current * fsw)
    return l_value * 1e6  # 转换为微亨


def calculate_output_capacitor(vout, iout_max, fsw_khz, voltage_ripple_ratio=0.01):
    """计算输出电容"""
    fsw = fsw_khz * 1000
    voltage_ripple = vout * voltage_ripple_ratio
    # 假设电感电流纹波为30%
    inductor_ripple_current = 0.3 * iout_max
    cout_min = inductor_ripple_current / (8 * fsw * voltage_ripple)
    return cout_min * 1e6  # 转换为微法


def calculate_input_capacitor(
    vin, iout_max, duty_cycle, fsw_khz, voltage_ripple_ratio=0.01
):
    """计算输入电容"""
    fsw = fsw_khz * 1000
    voltage_ripple = vin * voltage_ripple_ratio
    # 输入RMS电流
    i_rms = iout_max * math.sqrt(duty_cycle * (1 - duty_cycle))
    cin_min = i_rms / (fsw * voltage_ripple)
    return cin_min * 1e6  # 转换为微法


def calculate_bootstrap_capacitor(gate_charge=10e-9):
    """计算自举电容"""
    # 典型值为0.1uF，但可以根据MOSFET栅极电荷计算
    cboot = gate_charge / 5  # 假设5V的栅极驱动电压
    return max(cboot * 1e6, 0.1)  # 转换为微法，最小0.1uF


def calculate_soft_start_capacitor(startup_time_ms, current_source=10e-6):
    """计算软启动电容"""
    # 典型的软启动电流源为10uA
    css = (current_source * startup_time_ms * 1e-3) / 0.7  # 0.7V是典型的阈值电压
    return css * 1e6  # 转换为微法


def calculate_compensation_capacitor(
    crossover_freq_khz, error_amp_gm=200e-6, rfbt=100e3, rfbb=10e3
):
    """计算补偿电容"""
    # 简化计算，实际应根据环路分析确定
    crossover_freq = crossover_freq_khz * 1000
    feedback_divider = rfbb / (rfbt + rfbb)
    ccomp = 1 / (
        2 * math.pi * crossover_freq * (rfbt * feedback_divider) / error_amp_gm
    )
    return ccomp * 1e9  # 转换为纳法


def main():
    # 输入参数
    print("===== 降压转换器参数计算器 =====")
    print("\n请输入以下参数:")

    vin = float(input("输入电压 (V): "))
    vout = float(input("输出电压 (V): "))
    iout_max = float(input("最大输出电流 (A): "))
    fsw_khz = float(input("开关频率 (kHz): "))
    startup_time_ms = float(input("软启动时间 (ms): "))

    # 计算占空比
    duty_cycle = vout / vin

    # 计算各组件参数
    rt_ohm = calculate_rt(fsw_khz)
    rfbt, rfbb = calculate_feedback_resistors(vout)
    l_value_uh = calculate_inductor(vin, vout, iout_max, fsw_khz)
    cout_uf = calculate_output_capacitor(vout, iout_max, fsw_khz)
    cin_uf = calculate_input_capacitor(vin, iout_max, duty_cycle, fsw_khz)
    cboot_uf = calculate_bootstrap_capacitor()
    css_uf = calculate_soft_start_capacitor(startup_time_ms)
    ccomp_nf = calculate_compensation_capacitor(fsw_khz / 10, rfbt=rfbt, rfbb=rfbb)

    # 显示结果
    print("\n===== 计算结果 =====")
    print(
        f"RT = {rt_ohm/1000:.2f} kΩ (最接近的标准值: {find_closest_standard_value(rt_ohm/1000)} kΩ)"
    )
    print(
        f"RFBT = {rfbt/1000:.2f} kΩ (最接近的标准值: {find_closest_standard_value(rfbt/1000)} kΩ)"
    )
    print(
        f"RFBB = {rfbb/1000:.2f} kΩ (最接近的标准值: {find_closest_standard_value(rfbb/1000)} kΩ)"
    )
    print(
        f"L = {l_value_uh:.2f} μH (最接近的标准值: {find_closest_inductor_value(l_value_uh)} μH)"
    )
    print(f"COUT = {cout_uf:.2f} μF (建议值: {suggest_capacitor_value(cout_uf)} μF)")
    print(f"CIN = {cin_uf:.2f} μF (建议值: {suggest_capacitor_value(cin_uf)} μF)")
    print(f"CBOOT = {cboot_uf:.2f} μF (标准值: 0.1 μF)")
    print(
        f"CSS = {css_uf:.2f} μF (最接近的标准值: {find_closest_capacitor_value(css_uf)} μF)"
    )
    print(
        f"CCOMP = {ccomp_nf:.2f} nF (最接近的标准值: {find_closest_capacitor_value(ccomp_nf/1000)*1000} nF)"
    )

    # 额外信息
    print("\n===== 设计参数 =====")
    print(f"占空比 = {duty_cycle*100:.1f}%")
    print(f"电感电流纹波 = {0.3*iout_max:.2f} A (峰峰值，约为最大输出电流的30%)")
    print(f"输出电压纹波 = {vout*0.01:.3f} V (峰峰值，约为输出电压的1%)")

    # 绘制效率曲线（估算值）
    plot_efficiency_curve(vin, vout, iout_max, fsw_khz)


def find_closest_standard_value(value):
    """找到最接近的E24系列标准电阻值"""
    e24_series = [
        1.0,
        1.1,
        1.2,
        1.3,
        1.5,
        1.6,
        1.8,
        2.0,
        2.2,
        2.4,
        2.7,
        3.0,
        3.3,
        3.6,
        3.9,
        4.3,
        4.7,
        5.1,
        5.6,
        6.2,
        6.8,
        7.5,
        8.2,
        9.1,
    ]

    decade = 10 ** math.floor(math.log10(value))
    normalized = value / decade

    closest = min(e24_series, key=lambda x: abs(x - normalized))
    return closest * decade


def find_closest_inductor_value(value_uh):
    """找到最接近的标准电感值"""
    standard_inductors = [
        0.1,
        0.22,
        0.33,
        0.47,
        0.68,
        1.0,
        1.5,
        2.2,
        3.3,
        4.7,
        6.8,
        10,
        15,
        22,
        33,
        47,
        68,
        100,
        150,
        220,
        330,
        470,
        680,
        1000,
    ]

    if value_uh >= 1000:
        # 对于大于等于1000μH的值，转换为mH单位
        return f"{find_closest_standard_value(value_uh/1000)}m"
    else:
        closest = min(standard_inductors, key=lambda x: abs(x - value_uh))
        return closest


def suggest_capacitor_value(value_uf):
    """建议电容值，考虑并联和安全余量"""
    standard_caps = [0.1, 0.22, 0.47, 1.0, 2.2, 4.7, 10, 22, 47, 100, 220, 470, 1000]

    # 增加50%安全余量
    target = value_uf * 1.5

    if target <= max(standard_caps):
        # 找到大于等于目标值的最小标准值
        for cap in sorted(standard_caps):
            if cap >= target:
                return cap
    else:
        # 需要并联电容
        # 找到最接近的标准值组合
        best_combo = []
        remaining = target

        while remaining > 0.1:  # 0.1μF是最小考虑的电容
            closest = min(standard_caps, key=lambda x: abs(x - remaining))
            best_combo.append(closest)
            remaining -= closest
            if len(best_combo) >= 3:  # 最多考虑3个并联
                break

        if len(best_combo) == 1:
            return best_combo[0]
        else:
            return f"{sum(best_combo)} ({' + '.join(map(str, best_combo))})"


def find_closest_capacitor_value(value_uf):
    """找到最接近的标准电容值"""
    standard_caps = [
        0.001,
        0.0022,
        0.0047,
        0.01,
        0.022,
        0.047,
        0.1,
        0.22,
        0.47,
        1.0,
        2.2,
        4.7,
        10,
        22,
        47,
        100,
        220,
        470,
        1000,
    ]

    closest = min(standard_caps, key=lambda x: abs(x - value_uf))
    return closest


def plot_efficiency_curve(vin, vout, iout_max, fsw_khz):
    """绘制估算的效率曲线"""
    # 简化的效率模型
    current_points = np.linspace(0.05, iout_max, 20)
    efficiencies = []

    # 估计参数
    rds_on = 0.1  # 假设的MOSFET导通电阻
    fsw = fsw_khz * 1000
    duty_cycle = vout / vin

    for iout in current_points:
        # 导通损耗
        p_conduction = iout**2 * rds_on * duty_cycle

        # 开关损耗 (简化模型)
        p_switching = vin * iout * fsw * 20e-9  # 假设20ns的开关时间

        # 控制器损耗 (固定值)
        p_control = 0.1

        # 总损耗
        p_loss = p_conduction + p_switching + p_control

        # 输出功率
        p_out = vout * iout

        # 输入功率
        p_in = p_out + p_loss

        # 效率
        efficiency = (p_out / p_in) * 100 if p_in > 0 else 0
        efficiencies.append(efficiency)

    plt.figure(figsize=(10, 6))
    plt.plot(current_points, efficiencies, "b-", linewidth=2)
    plt.grid(True)
    plt.xlabel("输出电流 (A)")
    plt.ylabel("效率 (%)")
    plt.title(f"降压转换器估算效率曲线 (Vin={vin}V, Vout={vout}V, Fsw={fsw_khz}kHz)")
    plt.xlim(0, iout_max * 1.1)
    plt.ylim(70, 100)
    plt.show()


if __name__ == "__main__":
    main()
