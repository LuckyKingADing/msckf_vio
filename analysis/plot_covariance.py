#!/usr/bin/env python3
"""
协方差分析
- 位置协方差对角元素（σ_x, σ_y, σ_z）
- 姿态协方差对角元素（σ_roll, σ_pitch, σ_yaw）
- 速度协方差
- 协方差热力图快照
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

    # 检查协方差列是否存在
    if 'pose_cov_0' not in df.columns:
        print("  [SKIP] 无协方差数据")
        return

    # 位姿协方差是 6x6 矩阵（按行展开 0-35）
    # 对角线: [0, 7, 14, 21, 28, 35]
    # 位置: 对角线 0, 7, 14 对应 x, y, z
    # 姿态: 对角线 21, 28, 35 对应 roll, pitch, yaw
    pose_diag_idx = [0, 7, 14, 21, 28, 35]
    pose_labels = ['σ²_x', 'σ²_y', 'σ²_z', 'σ²_roll', 'σ²_pitch', 'σ²_yaw']

    # ======== 图1: 位置协方差 (标准差) ========
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    colors = ['#e74c3c', '#2ecc71', '#3498db']
    pos_names = ['σ_x', 'σ_y', 'σ_z']

    for i, (ax, color, name) in enumerate(zip(axes, colors, pos_names)):
        col = f'pose_cov_{pose_diag_idx[i]}'
        variance = df[col].values
        # 取绝对值后开方（协方差可能有数值噪声导致负值）
        std = np.sqrt(np.abs(variance))
        ax.plot(t, std, color=color, linewidth=1.0)
        ax.fill_between(t, 0, std, alpha=0.2, color=color)
        ax.set_ylabel(f'{name} (m)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend([name], loc='upper right')
        ax.text(0.02, 0.85, f'Mean: {std.mean():.6f}\nMax: {std.max():.6f}',
                transform=ax.transAxes, fontsize=9,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.suptitle('MSCKF-VIO Position Standard Deviation', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'covariance_position.png'), dpi=150)
    plt.close(fig)
    print("  -> covariance_position.png")

    # ======== 图2: 姿态协方差 (标准差) ========
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    ori_names = ['σ_roll', 'σ_pitch', 'σ_yaw']

    for i, (ax, color, name) in enumerate(zip(axes, colors, ori_names)):
        col = f'pose_cov_{pose_diag_idx[i+3]}'
        variance = df[col].values
        std_rad = np.sqrt(np.abs(variance))
        std_deg = np.degrees(std_rad)
        ax.plot(t, std_deg, color=color, linewidth=1.0)
        ax.fill_between(t, 0, std_deg, alpha=0.2, color=color)
        ax.set_ylabel(f'{name} (°)', fontsize=12)
        ax.grid(True, alpha=0.3)
        ax.legend([name], loc='upper right')
        ax.text(0.02, 0.85, f'Mean: {std_deg.mean():.4f}°\nMax: {std_deg.max():.4f}°',
                transform=ax.transAxes, fontsize=9,
                bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    fig.suptitle('MSCKF-VIO Orientation Standard Deviation', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'covariance_orientation.png'), dpi=150)
    plt.close(fig)
    print("  -> covariance_orientation.png")

    # ======== 图3: 协方差热力图（取中间时刻）========
    mid = len(df) // 2
    cov_matrix = np.zeros((6, 6))
    for i in range(6):
        for j in range(6):
            col = f'pose_cov_{i*6+j}'
            if col in df.columns:
                cov_matrix[i, j] = df[col].iloc[mid]

    fig, ax = plt.subplots(figsize=(8, 7))
    im = ax.imshow(cov_matrix, cmap='RdBu_r', aspect='auto')
    ax.set_xticks(range(6))
    ax.set_yticks(range(6))
    labels = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_yticklabels(labels, fontsize=11)
    ax.set_title(f'Pose Covariance Matrix (t = {t[mid]:.1f}s)', fontsize=14)

    # 添加数值标注
    for i in range(6):
        for j in range(6):
            val = cov_matrix[i, j]
            ax.text(j, i, f'{val:.2e}', ha='center', va='center', fontsize=7,
                    color='white' if abs(val) > abs(cov_matrix).max()*0.5 else 'black')

    fig.colorbar(im, ax=ax, shrink=0.8)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'covariance_heatmap.png'), dpi=150)
    plt.close(fig)
    print("  -> covariance_heatmap.png")

    # ======== 图4: 速度协方差 ========
    twist_diag_idx = [0, 7, 14, 21, 28, 35]
    has_twist_cov = all(f'twist_cov_{i}' in df.columns for i in twist_diag_idx[:3])

    if has_twist_cov:
        fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)

        # 线速度协方差
        for i, (color, name) in enumerate(zip(colors, ['σ_vx', 'σ_vy', 'σ_vz'])):
            col = f'twist_cov_{twist_diag_idx[i]}'
            std = np.sqrt(np.abs(df[col].values))
            axes[0].plot(t, std, color=color, linewidth=0.8, label=name)

        axes[0].set_ylabel('σ (m/s)', fontsize=12)
        axes[0].set_title('Linear Velocity Std Dev', fontsize=12)
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # 角速度协方差
        for i, (color, name) in enumerate(zip(colors, ['σ_ωx', 'σ_ωy', 'σ_ωz'])):
            col = f'twist_cov_{twist_diag_idx[i+3]}'
            if col in df.columns:
                std = np.degrees(np.sqrt(np.abs(df[col].values)))
                axes[1].plot(t, std, color=color, linewidth=0.8, label=name)

        axes[1].set_ylabel('σ (°/s)', fontsize=12)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].set_title('Angular Velocity Std Dev', fontsize=12)
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        fig.suptitle('MSCKF-VIO Velocity Covariance', fontsize=14)
        fig.tight_layout()
        fig.savefig(os.path.join(data_dir, 'covariance_velocity.png'), dpi=150)
        plt.close(fig)
        print("  -> covariance_velocity.png")


if __name__ == "__main__":
    main()
