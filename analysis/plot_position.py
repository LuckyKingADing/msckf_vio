#!/usr/bin/env python3
"""
位置 XYZ 时间序列
- X/Y/Z 随时间变化曲线
- 位置变化率（数值微分）
"""
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data_loader import load_odom, ensure_output_dir


def main():
    data_dir = sys.argv[1]
    df = load_odom(data_dir)
    ensure_output_dir(data_dir)

    t = df['t'].values
    px, py, pz = df['px'].values, df['py'].values, df['pz'].values

    # ======== 图1: 位置时间序列 ========
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    colors = ['#e74c3c', '#2ecc71', '#3498db']
    labels = ['X', 'Y', 'Z']
    data = [px, py, pz]

    for ax, d, c, l in zip(axes, data, colors, labels):
        ax.plot(t, d, color=c, linewidth=1.0)
        ax.set_ylabel(f'{l} (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend([l], loc='upper right', fontsize=11)
        # 标注范围
        ax.text(0.02, 0.95, f'Range: [{d.min():.3f}, {d.max():.3f}]',
                transform=ax.transAxes, fontsize=9, va='top',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.suptitle('MSCKF-VIO Position over Time', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'position_xyz.png'), dpi=150)
    plt.close(fig)
    print("  -> position_xyz.png")

    # ======== 图2: 位置变化率 ========
    dt = np.diff(t)
    dt[dt == 0] = 1e-9  # 避免除零

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    for ax, d, c, l in zip(axes, data, colors, labels):
        rate = np.diff(d) / dt
        ax.plot(t[1:], rate, color=c, linewidth=0.8, alpha=0.8)
        ax.set_ylabel(f'd{l}/dt (m/s)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend([f'd{l}/dt'], loc='upper right', fontsize=11)

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.suptitle('MSCKF-VIO Position Rate of Change', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'position_rate.png'), dpi=150)
    plt.close(fig)
    print("  -> position_rate.png")


if __name__ == "__main__":
    main()
