#!/usr/bin/env python3
"""
3D / 2D 轨迹可视化
- 3D 轨迹
- XY 俯视图
- XZ 侧视图
- YZ 侧视图
"""
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data_loader import load_odom, ensure_output_dir


def main():
    data_dir = sys.argv[1]
    df = load_odom(data_dir)
    ensure_output_dir(data_dir)

    px, py, pz = df['px'].values, df['py'].values, df['pz'].values
    t = df['t'].values

    # ======== 图1: 3D 轨迹 ========
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    sc = ax.scatter(px, py, pz, c=t, cmap='viridis', s=1, alpha=0.8)
    ax.plot(px[0], py[0], pz[0], 'go', markersize=12, label='Start')
    ax.plot(px[-1], py[-1], pz[-1], 'r^', markersize=12, label='End')
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_zlabel('Z (m)', fontsize=12)
    ax.set_title('MSCKF-VIO 3D Trajectory', fontsize=14)
    ax.legend(fontsize=11)
    cbar = fig.colorbar(sc, ax=ax, shrink=0.6, pad=0.1)
    cbar.set_label('Time (s)', fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'trajectory_3d.png'), dpi=150)
    plt.close(fig)
    print("  -> trajectory_3d.png")

    # ======== 图2: 2D 投影（2x2 子图）========
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # XY 俯视图
    ax = axes[0, 0]
    ax.plot(px, py, 'b-', linewidth=0.8, alpha=0.7)
    ax.scatter(px, py, c=t, cmap='viridis', s=2, zorder=5)
    ax.plot(px[0], py[0], 'go', markersize=10, label='Start')
    ax.plot(px[-1], py[-1], 'r^', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY Plane (Top View)')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)

    # XZ 侧视图
    ax = axes[0, 1]
    ax.plot(px, pz, 'b-', linewidth=0.8, alpha=0.7)
    ax.scatter(px, pz, c=t, cmap='viridis', s=2, zorder=5)
    ax.plot(px[0], pz[0], 'go', markersize=10, label='Start')
    ax.plot(px[-1], pz[-1], 'r^', markersize=10, label='End')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('XZ Plane (Side View)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # YZ 侧视图
    ax = axes[1, 0]
    ax.plot(py, pz, 'b-', linewidth=0.8, alpha=0.7)
    ax.scatter(py, pz, c=t, cmap='viridis', s=2, zorder=5)
    ax.plot(py[0], pz[0], 'go', markersize=10, label='Start')
    ax.plot(py[-1], pz[-1], 'r^', markersize=10, label='End')
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('YZ Plane (Side View)')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 行程距离
    ax = axes[1, 1]
    dist = np.cumsum(np.sqrt(np.diff(px)**2 + np.diff(py)**2 + np.diff(pz)**2))
    dist = np.insert(dist, 0, 0)
    ax.plot(t, dist, 'g-', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Distance (m)')
    ax.set_title('Cumulative Travel Distance')
    ax.grid(True, alpha=0.3)
    ax.text(0.95, 0.05, f'Total: {dist[-1]:.2f} m',
            transform=ax.transAxes, ha='right', fontsize=12,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    fig.suptitle('MSCKF-VIO Trajectory Projections', fontsize=14, y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'trajectory_2d.png'), dpi=150)
    plt.close(fig)
    print("  -> trajectory_2d.png")


if __name__ == "__main__":
    main()
