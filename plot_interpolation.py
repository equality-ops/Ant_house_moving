import matplotlib
matplotlib.use('Agg')  # 非交互式后端，无需 GUI
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import font_manager
import os

# 尝试使用中文字体
try:
    # Windows 常见中文字体
    for f in ['Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC', 'WenQuanYi Micro Hei']:
        try:
            plt.rcParams['font.family'] = f
            break
        except:
            continue
except:
    pass

plt.rcParams['axes.unicode_minus'] = False

# ============================================================
# 插值函数定义
# ============================================================

def smoothstep(t, k=0.5):
    """当前的 Gentle-Smoothstep"""
    t = np.clip(t, 0.0, 1.0)
    cubic = 3 * t**2 - 2 * t**3
    return k * t + (1 - k) * cubic

def smoothstep_deriv(t, k=0.5):
    """smoothstep 的一阶导数"""
    t = np.clip(t, 0.0, 1.0)
    return k + (1 - k) * (6 * t - 6 * t**2)

def smoothstep_integral(t, k=0.5):
    """smoothstep 的积分 ∫f(τ)dτ from 0 to t"""
    t = np.clip(t, 0.0, 1.0)
    # ∫ kτ + (1-k)(3τ² - 2τ³) dτ = (k/2)t² + (1-k)(t³ - t⁴/2)
    return (k / 2) * t**2 + (1 - k) * (t**3 - 0.5 * t**4)


def ease_out(t, power=3.0):
    """缓出函数 f(t) = 1 - (1-t)^n"""
    t = np.clip(t, 0.0, 1.0)
    return 1.0 - (1.0 - t) ** power

def ease_out_deriv(t, power=3.0):
    """ease_out 的一阶导数 f'(t) = n * (1-t)^(n-1)"""
    t = np.clip(t, 0.0, 1.0)
    return power * (1.0 - t) ** (power - 1.0)

def ease_out_integral(t, power=3.0):
    """ease_out 的积分 ∫f(τ)dτ from 0 to t"""
    t = np.clip(t, 0.0, 1.0)
    # ∫ 1 - (1-τ)^n dτ = τ + (1-τ)^(n+1)/(n+1) | from 0 to t
    # = t + (1-t)^(n+1)/(n+1) - 1/(n+1)
    return t + (1.0 - t) ** (power + 1.0) / (power + 1.0) - 1.0 / (power + 1.0)


# ============================================================
# 绘图
# ============================================================
t = np.linspace(0, 1, 500)

# 要对比的曲线
configs = [
    # (label, 函数, 导数, 积分, 颜色, 线型)
    ("smoothstep k=0.5 (current)", smoothstep, smoothstep_deriv, smoothstep_integral, '#888888', '--'),
    ("ease-out n=2", ease_out, ease_out_deriv, ease_out_integral, '#2196F3', '-'),
    ("ease-out n=3", ease_out, ease_out_deriv, ease_out_integral, '#4CAF50', '-'),
    ("ease-out n=4", ease_out, ease_out_deriv, ease_out_integral, '#FF5722', '-'),
    ("ease-out n=5", ease_out, ease_out_deriv, ease_out_integral, '#9C27B0', '-'),
]

colors = [c[4] for c in configs]
linestyles = [c[5] for c in configs]
labels = [c[0] for c in configs]

# 先分别计算各 power 的 ease_out
power_funcs = [
    ("smoothstep k=0.5", lambda t: smoothstep(t, 0.5), lambda t: smoothstep_deriv(t, 0.5), lambda t: smoothstep_integral(t, 0.5)),
    ("ease-out n=2", lambda t: ease_out(t, 2), lambda t: ease_out_deriv(t, 2), lambda t: ease_out_integral(t, 2)),
    ("ease-out n=3", lambda t: ease_out(t, 3), lambda t: ease_out_deriv(t, 3), lambda t: ease_out_integral(t, 3)),
    ("ease-out n=4", lambda t: ease_out(t, 4), lambda t: ease_out_deriv(t, 4), lambda t: ease_out_integral(t, 4)),
    ("ease-out n=5", lambda t: ease_out(t, 5), lambda t: ease_out_deriv(t, 5), lambda t: ease_out_integral(t, 5)),
]

# 颜色方案
palette = ['#888888', '#2196F3', '#4CAF50', '#FF5722', '#9C27B0']
linestyles_list = ['--', '-', '-', '-', '-']

fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))

# ---- 图1: 速度曲线 f(t) ----
ax = axes[0]
for (name, fn, _, _), c, ls in zip(power_funcs, palette, linestyles_list):
    ax.plot(t, fn(t), color=c, linestyle=ls, linewidth=2, label=name)
ax.set_title("Speed Curve f(t)", fontsize=13, fontweight='bold')
ax.set_xlabel("Normalized Distance t (s / d_acc)")
ax.set_ylabel("Speed Factor")
ax.set_xlim(0, 1)
ax.set_ylim(0, 1.05)
ax.legend(fontsize=8, loc='lower right')
ax.grid(True, alpha=0.3)
# 标注关键区域
ax.axvspan(0, 0.3, alpha=0.06, color='red')
ax.annotate("起步阶段\n(重点关注)", xy=(0.15, 0.15), fontsize=9, color='red', ha='center')

# ---- 图2: 加速度曲线 f'(t) ----
ax = axes[1]
for (name, _, fder, _), c, ls in zip(power_funcs, palette, linestyles_list):
    ax.plot(t, fder(t), color=c, linestyle=ls, linewidth=2, label=name)
ax.set_title("Acceleration Curve f'(t)", fontsize=13, fontweight='bold')
ax.set_xlabel("Normalized Distance t (s / d_acc)")
ax.set_ylabel("Acceleration Factor")
ax.set_xlim(0, 1)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)
ax.axvspan(0, 0.3, alpha=0.06, color='red')

# ---- 图3: 位移曲线 ∫f(t) ----
ax = axes[2]
for (name, _, _, fint), c, ls in zip(power_funcs, palette, linestyles_list):
    ax.plot(t, fint(t), color=c, linestyle=ls, linewidth=2, label=name)
ax.set_title("Cumulative Displacement ∫f(τ)dτ", fontsize=13, fontweight='bold')
ax.set_xlabel("Normalized Distance t (s / d_acc)")
ax.set_ylabel("Cumulative Displacement")
ax.set_xlim(0, 1)
ax.legend(fontsize=8, loc='lower right')
ax.grid(True, alpha=0.3)

# 总标题
fig.suptitle("Interpolation Function Comparison — Ease-Out vs Smoothstep",
             fontsize=15, fontweight='bold', y=1.01)

plt.tight_layout()
output_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "interpolation_comparison.png")
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"图表已保存到: {output_path}")
