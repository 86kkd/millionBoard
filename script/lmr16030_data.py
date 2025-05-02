import math
import numpy as np
import matplotlib.pyplot as plt
import argparse


def calculate_rt(fsw_khz):
    """计算RT电阻值(欧姆)，基于LMR16030/LMR16020系列芯片"""
    rt_ohm = 42904 / (fsw_khz**1.088)
    return rt_ohm * 1000


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
    curren_ripple = 0.9 * iout_max
    cout_min = 3 * curren_ripple / fsw / voltage_ripple
    return cout_min * 1e5  # 转换为微法


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


def calculate_soft_start_capacitor(startup_time_ms, current_source=3e-6):
    """计算软启动电容"""
    # 典型的软启动电流源为3uA
    css = (current_source * startup_time_ms * 1e-3) / 0.7  # 0.7V是典型的阈值电压
    return css * 1e6  # 转换为微法


def main():
    # 使用argparse解析命令行参数
    parser = argparse.ArgumentParser(
        description="降压转换器参数计算器 (基于LMR16030/LMR16020)"
    )
    parser.add_argument("--vin", type=float, required=True, help="输入电压 (V)")
    parser.add_argument("--vout", type=float, required=True, help="输出电压 (V)")
    parser.add_argument(
        "--iout_max", type=float, required=True, help="最大输出电流 (A)"
    )
    parser.add_argument("--fsw_khz", type=float, required=True, help="开关频率 (kHz)")
    parser.add_argument(
        "--startup_time_ms", type=float, required=True, help="软启动时间 (ms)"
    )
    parser.add_argument("--show_graph", action="store_true", help="显示效率曲线图")

    args = parser.parse_args()

    # 获取参数
    vin = args.vin
    vout = args.vout
    iout_max = args.iout_max
    fsw_khz = args.fsw_khz
    startup_time_ms = args.startup_time_ms
    show_graph = args.show_graph

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

    # 额外信息
    print("\n===== 设计参数 =====")
    print(f"占空比 = {duty_cycle*100:.1f}%")
    print(f"电感电流纹波 = {0.3*iout_max:.2f} A (峰峰值，约为最大输出电流的30%)")
    print(f"输出电压纹波 = {vout*0.01:.3f} V (峰峰值，约为输出电压的1%)")

    # 绘制效率曲线（估算值）
    if show_graph:
        plot_efficiency_curve(vin, vout, iout_max, fsw_khz)
    else:
        # 即使不显示图形，也打印效率数据
        print_efficiency_data(vin, vout, iout_max, fsw_khz)


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


def print_efficiency_data(
    vin, vout, iout_max, fsw_khz, typical_load_min=0.2, typical_load_max=0.8
):
    """打印效率数据但不显示图形"""
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

    # 计算典型负载区间
    typical_min_current = iout_max * typical_load_min
    typical_max_current = iout_max * typical_load_max

    # 找到典型负载区间对应的效率值
    min_idx = np.abs(current_points - typical_min_current).argmin()
    max_idx = np.abs(current_points - typical_max_current).argmin()

    typical_min_efficiency = efficiencies[min_idx]
    typical_max_efficiency = efficiencies[max_idx]

    # 计算最大效率及其对应的电流点
    max_efficiency = max(efficiencies)
    max_eff_idx = efficiencies.index(max_efficiency)
    max_eff_current = current_points[max_eff_idx]

    # 打印效率数据表
    print("\n===== 效率数据 =====")
    print("负载百分比  |  输出电流  |  效率")
    print("------------|-----------|--------")

    load_percentages = [10, 25, 50, 75, 100]
    for pct in load_percentages:
        current = iout_max * pct / 100
        idx = np.abs(current_points - current).argmin()
        efficiency = efficiencies[idx]
        print(f"{pct:12}% | {current:8.2f}A | {efficiency:6.1f}%")

    # 打印典型工作区间的效率范围
    print(f"\n典型工作区间 ({typical_load_min*100}%-{typical_load_max*100}% 负载):")
    print(f"  - 电流范围: {typical_min_current:.2f}A - {typical_max_current:.2f}A")
    print(
        f"  - 效率范围: {min(typical_min_efficiency, typical_max_efficiency):.1f}% - {max(typical_min_efficiency, typical_max_efficiency):.1f}%"
    )
    print(f"最大效率: {max_efficiency:.1f}% (在 {max_eff_current:.2f}A 时)")


def plot_efficiency_curve(
    vin, vout, iout_max, fsw_khz, typical_load_min=0.2, typical_load_max=0.8
):
    """
    绘制估算的效率曲线，并标注正常工作区间和效率数值

    参数:
    vin - 输入电压 (V)
    vout - 输出电压 (V)
    iout_max - 最大输出电流 (A)
    fsw_khz - 开关频率 (kHz)
    typical_load_min - 典型负载最小值 (占最大负载的百分比)
    typical_load_max - 典型负载最大值 (占最大负载的百分比)
    """
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

    # 创建图表
    fig, ax = plt.figure(figsize=(10, 6)), plt.gca()

    # 绘制效率曲线
    plt.plot(current_points, efficiencies, "b-", linewidth=2)

    # 计算典型负载区间
    typical_min_current = iout_max * typical_load_min
    typical_max_current = iout_max * typical_load_max

    # 找到典型负载区间对应的效率值
    min_idx = np.abs(current_points - typical_min_current).argmin()
    max_idx = np.abs(current_points - typical_max_current).argmin()

    typical_min_efficiency = efficiencies[min_idx]
    typical_max_efficiency = efficiencies[max_idx]

    # 计算最大效率及其对应的电流点
    max_efficiency = max(efficiencies)
    max_eff_idx = efficiencies.index(max_efficiency)
    max_eff_current = current_points[max_eff_idx]

    # Mark typical operating range
    rect = Rectangle(
        (typical_min_current, 70),
        typical_max_current - typical_min_current,
        30,
        facecolor="lightgreen",
        alpha=0.3,
        label="Typical Operating Range",
    )
    ax.add_patch(rect)

    # Annotate efficiency values on the chart
    plt.annotate(
        f"{typical_min_efficiency:.1f}%",
        xy=(typical_min_current, typical_min_efficiency),
        xytext=(typical_min_current - 0.1, typical_min_efficiency + 3),
        arrowprops=dict(arrowstyle="->", color="red"),
    )

    plt.annotate(
        f"{typical_max_efficiency:.1f}%",
        xy=(typical_max_current, typical_max_efficiency),
        xytext=(typical_max_current + 0.1, typical_max_efficiency + 3),
        arrowprops=dict(arrowstyle="->", color="red"),
    )

    plt.annotate(
        f"Max Efficiency: {max_efficiency:.1f}%",
        xy=(max_eff_current, max_efficiency),
        xytext=(max_eff_current, max_efficiency - 5),
        arrowprops=dict(arrowstyle="->", color="red"),
        fontweight="bold",
    )

    # Add chart elements
    plt.grid(True)
    plt.xlabel("Output Current (A)")
    plt.ylabel("Efficiency (%)")
    plt.title(
        f"Buck Converter Efficiency Curve (Vin={vin}V, Vout={vout}V, Fsw={fsw_khz}kHz)"
    )
    plt.xlim(0, iout_max * 1.1)
    plt.ylim(70, 100)

    # Add legend
    plt.legend(loc="lower right")

    # Add efficiency data table
    table_data = []
    load_percentages = [10, 25, 50, 75, 100]

    for pct in load_percentages:
        current = iout_max * pct / 100
        idx = np.abs(current_points - current).argmin()
        efficiency = efficiencies[idx]
        table_data.append([f"{pct}%", f"{current:.2f}A", f"{efficiency:.1f}%"])

    # Add table
    plt.table(
        cellText=table_data,
        colLabels=["Load Percentage", "Output Current", "Efficiency"],
        loc="bottom",
        bbox=[0.15, -0.35, 0.7, 0.2],
    )

    plt.subplots_adjust(bottom=0.25)  # Make room for the table

    # Display chart
    plt.show()

    # 打印典型工作区间的效率范围
    print(f"典型工作区间 ({typical_load_min*100}%-{typical_load_max*100}% 负载):")
    print(f"  - 电流范围: {typical_min_current:.2f}A - {typical_max_current:.2f}A")
    print(
        f"  - 效率范围: {min(typical_min_efficiency, typical_max_efficiency):.1f}% - {max(typical_min_efficiency, typical_max_efficiency):.1f}%"
    )
    print(f"最大效率: {max_efficiency:.1f}% (在 {max_eff_current:.2f}A 时)")


if __name__ == "__main__":
    main()
