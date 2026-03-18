#!/usr/bin/env python3
"""
速度分析
- 线速度 (vx, vy, vz) 时间序列
- 线速度合成 (speed)
- 角速度 (wx, wy, wz) 时间序列
- 角速度合成
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

    # 检查速度列是否存在
    if 'vx' not in df.columns:
        print("  [SKIP] 无速度数据")
        return

    vx, vy, vz = df['vx'].values, df['vy'].values, df['vz'].values
    wx, wy, wz = df['wx'].values, df['wy'].values, df['wz'].values

    speed = np.sqrt(vx**2 + vy**2 + vz**2)
    angular_speed = np.sqrt(wx**2 + wy**2 + wz**2)

    # ======== 图1: 线速度 ========
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    axes[0].plot(t, vx, '#e74c3c', linewidth=0.8)
    axes[0].set_ylabel('vx (m/s)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(['vx'], loc='upper right')

    axes[1].plot(t, vy, '#2ecc71', linewidth=0.8)
    axes[1].set_ylabel('vy (m/s)')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(['vy'], loc='upper right')

    axes[2].plot(t, vz, '#3498db', linewidth=0.8)
    axes[2].set_ylabel('vz (m/s)')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(['vz'], loc='upper right')

    axes[3].plot(t, speed, '#8e44ad', linewidth=1.0)
    axes[3].fill_between(t, 0, speed, alpha=0.2, color='#8e44ad')
    axes[3].set_ylabel('Speed (m/s)')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(['|v|'], loc='upper right')
    axes[3].text(0.02, 0.85, f'Mean: {speed.mean():.3f} m/s\nMax: {speed.max():.3f} m/s',
                 transform=axes[3].transAxes, fontsize=10,
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    fig.suptitle('MSCKF-VIO Linear Velocity', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'velocity_linear.png'), dpi=150)
    plt.close(fig)
    print("  -> velocity_linear.png")

    # ======== 图2: 角速度 ========
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    axes[0].plot(t, wx, '#e74c3c', linewidth=0.8)
    axes[0].set_ylabel('ωx (rad/s)')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(['ωx'], loc='upper right')

    axes[1].plot(t, wy, '#2ecc71', linewidth=0.8)
    axes[1].set_ylabel('ωy (rad/s)')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(['ωy'], loc='upper right')

    axes[2].plot(t, wz, '#3498db', linewidth=0.8)
    axes[2].set_ylabel('ωz (rad/s)')
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(['ωz'], loc='upper right')

    axes[3].plot(t, angular_speed, '#e67e22', linewidth=1.0)
    axes[3].fill_between(t, 0, angular_speed, alpha=0.2, color='#e67e22')
    axes[3].set_ylabel('|ω| (rad/s)')
    axes[3].set_xlabel('Time (s)')
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(['|ω|'], loc='upper right')
    axes[3].text(0.02, 0.85, f'Mean: {angular_speed.mean():.3f} rad/s\nMax: {angular_speed.max():.3f} rad/s',
                 transform=axes[3].transAxes, fontsize=10,
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    fig.suptitle('MSCKF-VIO Angular Velocity', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'velocity_angular.png'), dpi=150)
    plt.close(fig)
    print("  -> velocity_angular.png")

    # ======== 图3: 速度概览（速度+角速度合在一张图）========
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

    ax1.plot(t, speed, '#8e44ad', linewidth=1.0)
    ax1.fill_between(t, 0, speed, alpha=0.15, color='#8e44ad')
    ax1.set_ylabel('Linear Speed (m/s)', fontsize=12)
    ax1.set_title('Speed Overview', fontsize=14)
    ax1.grid(True, alpha=0.3)

    ax2.plot(t, np.degrees(angular_speed), '#e67e22', linewidth=1.0)
    ax2.fill_between(t, 0, np.degrees(angular_speed), alpha=0.15, color='#e67e22')
    ax2.set_ylabel('Angular Speed (°/s)', fontsize=12)
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'velocity_overview.png'), dpi=150)
    plt.close(fig)
    print("  -> velocity_overview.png")


if __name__ == "__main__":
    main()
