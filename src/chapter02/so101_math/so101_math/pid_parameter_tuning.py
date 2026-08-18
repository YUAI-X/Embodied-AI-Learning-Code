"""通过一个简化关节模型直观看懂 Kp、Ki、Kd 的作用。"""

import argparse
import numpy as np
import matplotlib.pyplot as plt

from .plotting import configure_chinese_font


def simulate(kp, ki, kd, target=1.0, seconds=5.0, dt=0.01):
    """模拟简化二阶关节。返回时间、位置和控制量。"""
    steps = int(seconds / dt)
    time = np.arange(steps) * dt
    position = np.zeros(steps)
    velocity = 0.0
    integral = 0.0
    previous_error = target
    control_history = np.zeros(steps)

    for index in range(1, steps):
        error = target - position[index - 1]
        integral = np.clip(integral + error * dt, -2.0, 2.0)  # 简单抗积分饱和
        derivative = (error - previous_error) / dt
        control = kp * error + ki * integral + kd * derivative
        control = float(np.clip(control, -8.0, 8.0))

        # 简化动力学：加速度 = 控制量 - 阻尼×速度 - 小的恒定负载。
        acceleration = control - 0.8 * velocity - 0.15
        velocity += acceleration * dt
        position[index] = position[index - 1] + velocity * dt
        control_history[index] = control
        previous_error = error
    return time, position, control_history


def main():
    """解析可调PID参数，运行简化关节仿真并绘制响应曲线。"""
    selected_font = configure_chinese_font()
    parser = argparse.ArgumentParser(description="调节 PID 参数并观察关节响应")
    parser.add_argument("--kp", type=float, default=3.0, help="比例增益")
    parser.add_argument("--ki", type=float, default=0.0, help="积分增益")
    parser.add_argument("--kd", type=float, default=0.2, help="微分增益")
    parser.add_argument("--target", type=float, default=1.0, help="目标位置（弧度）")
    parser.add_argument("--save", default="", help="将曲线保存到指定文件")
    parser.add_argument("--no-show", action="store_true", help="不弹出图形窗口，适合自动测试")
    args = parser.parse_args()

    time, position, control = simulate(args.kp, args.ki, args.kd, args.target)
    error = args.target - position
    overshoot = max(0.0, float(np.max(position) - args.target))
    steady_error = float(np.mean(np.abs(error[-50:])))
    print(f"PID 参数：Kp={args.kp:.3f}, Ki={args.ki:.3f}, Kd={args.kd:.3f}")
    if selected_font is None:
        print("提示：未找到中文字体，图中文字可能显示异常；请安装Noto Sans CJK。")
    else:
        print(f"绘图字体：{selected_font}")
    print(f"最大超调量：{overshoot:.4f} rad")
    print(f"末段平均误差：{steady_error:.4f} rad")

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(8, 6))
    axes[0].plot(time, position, label="实际位置")
    axes[0].axhline(args.target, color="tab:red", linestyle="--", label="目标位置")
    axes[0].set_ylabel("位置 / rad")
    axes[0].legend()
    axes[0].grid(True)
    axes[1].plot(time, control, color="tab:green")
    axes[1].set_ylabel("控制量")
    axes[1].set_xlabel("时间 / s")
    axes[1].grid(True)
    fig.suptitle(f"PID 响应：Kp={args.kp}, Ki={args.ki}, Kd={args.kd}")
    fig.tight_layout()
    if args.save:
        fig.savefig(args.save, dpi=140)
        print(f"曲线已保存到：{args.save}")
    if not args.no_show:
        plt.show()
