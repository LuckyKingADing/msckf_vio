#!/usr/bin/env python3
"""
姿态分析
- 四元数时间序列
- 欧拉角（Roll, Pitch, Yaw）时间序列
"""
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data_loader import load_odom, quat_to_euler, ensure_output_dir


def main():
    data_dir = sys.argv[1]
    df = load_odom(data_dir)
    ensure_output_dir(data_dir)

    t = df['t'].values
    qx, qy, qz, qw = df['qx'].values, df['qy'].values, df['qz'].values, df['qw'].values

    # ======== 图1: 四元数 ========
    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

    components = [('qx', qx, '#e74c3c'), ('qy', qy, '#2ecc71'),
                  ('qz', qz, '#3498db'), ('qw', qw, '#9b59b6')]

    for ax, (name, data, color) in zip(axes, components):
        ax.plot(t, data, color=color, linewidth=1.0)
        ax.set_ylabel(name, fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend([name], loc='upper right')

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.suptitle('MSCKF-VIO Quaternion over Time', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'orientation_quaternion.png'), dpi=150)
    plt.close(fig)
    print("  -> orientation_quaternion.png")

    # ======== 图2: 欧拉角 ========
    roll, pitch, yaw = quat_to_euler(qx, qy, qz, qw)

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    euler_data = [('Roll', roll, '#e74c3c'),
                  ('Pitch', pitch, '#2ecc71'),
                  ('Yaw', yaw, '#3498db')]

    for ax, (name, data, color) in zip(axes, euler_data):
        ax.plot(t, data, color=color, linewidth=1.0)
        ax.set_ylabel(f'{name} (°)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend([name], loc='upper right')
        ax.text(0.02, 0.95, f'Range: [{data.min():.2f}°, {data.max():.2f}°]',
                transform=ax.transAxes, fontsize=9, va='top',
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.suptitle('MSCKF-VIO Euler Angles over Time', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'orientation_euler.png'), dpi=150)
    plt.close(fig)
    print("  -> orientation_euler.png")

    # ======== 图3: 四元数模（检验归一化）========
    quat_norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)

    fig, ax = plt.subplots(figsize=(14, 4))
    ax.plot(t, quat_norm, 'k-', linewidth=1.0)
    ax.axhline(y=1.0, color='r', linestyle='--', alpha=0.5, label='Ideal (1.0)')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('||q||', fontsize=12)
    ax.set_title('Quaternion Norm (should be ≈ 1.0)', fontsize=14)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0.99, 1.01)
    ax.text(0.02, 0.85, f'Mean: {quat_norm.mean():.6f}\nStd: {quat_norm.std():.2e}',
            transform=ax.transAxes, fontsize=10,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'quaternion_norm.png'), dpi=150)
    plt.close(fig)
    print("  -> quaternion_norm.png")


if __name__ == "__main__":
    main()
