#!/usr/bin/env python3
"""
汇总报告生成
- 统计摘要文本
- 综合概览图
"""
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data_loader import load_odom, load_tracking, quat_to_euler, ensure_output_dir


def main():
    data_dir = sys.argv[1]
    df = load_odom(data_dir)
    df_track = load_tracking(data_dir)
    ensure_output_dir(data_dir)

    t = df['t'].values
    px, py, pz = df['px'].values, df['py'].values, df['pz'].values

    # 基本统计
    duration = t[-1] - t[0]
    n_frames = len(df)
    avg_hz = n_frames / duration if duration > 0 else 0

    # 行程距离
    dist = np.sum(np.sqrt(np.diff(px)**2 + np.diff(py)**2 + np.diff(pz)**2))

    # 位置范围
    x_range = px.max() - px.min()
    y_range = py.max() - py.min()
    z_range = pz.max() - pz.min()

    # 速度统计
    speed = np.sqrt(df['vx'].values**2 + df['vy'].values**2 + df['vz'].values**2) if 'vx' in df.columns else None

    # 姿态范围
    roll, pitch, yaw = quat_to_euler(df['qx'].values, df['qy'].values,
                                      df['qz'].values, df['qw'].values)

    # 生成文本报告
    report = []
    report.append("=" * 60)
    report.append("MSCKF-VIO 分析报告")
    report.append(f"生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append(f"数据目录: {data_dir}")
    report.append("=" * 60)
    report.append("")
    report.append("【基本信息】")
    report.append(f"  总时长:           {duration:.2f} s ({duration/60:.1f} min)")
    report.append(f"  总帧数:           {n_frames}")
    report.append(f"  平均频率:         {avg_hz:.1f} Hz")
    report.append(f"  累积行程:         {dist:.2f} m")
    report.append("")
    report.append("【位置统计】")
    report.append(f"  X 范围:           [{px.min():.3f}, {px.max():.3f}] m  (跨度: {x_range:.3f} m)")
    report.append(f"  Y 范围:           [{py.min():.3f}, {py.max():.3f}] m  (跨度: {y_range:.3f} m)")
    report.append(f"  Z 范围:           [{pz.min():.3f}, {pz.max():.3f}] m  (跨度: {z_range:.3f} m)")
    report.append(f"  起点:             ({px[0]:.3f}, {py[0]:.3f}, {pz[0]:.3f})")
    report.append(f"  终点:             ({px[-1]:.3f}, {py[-1]:.3f}, {pz[-1]:.3f})")
    drift = np.sqrt((px[-1]-px[0])**2 + (py[-1]-py[0])**2 + (pz[-1]-pz[0])**2)
    report.append(f"  首尾距离:         {drift:.3f} m")
    if dist > 0:
        report.append(f"  首尾漂移率:       {drift/dist*100:.2f}%")
    report.append("")
    report.append("【姿态统计】 (欧拉角)")
    report.append(f"  Roll 范围:        [{roll.min():.2f}°, {roll.max():.2f}°]")
    report.append(f"  Pitch 范围:       [{pitch.min():.2f}°, {pitch.max():.2f}°]")
    report.append(f"  Yaw 范围:         [{yaw.min():.2f}°, {yaw.max():.2f}°]")

    if speed is not None:
        report.append("")
        report.append("【速度统计】")
        report.append(f"  平均线速度:       {speed.mean():.3f} m/s")
        report.append(f"  最大线速度:       {speed.max():.3f} m/s")
        report.append(f"  速度标准差:       {speed.std():.3f} m/s")

    if df_track is not None:
        before = df_track['before_tracking'].values
        after_ransac = df_track['after_ransac'].values
        overall = np.where(before > 0, after_ransac / before * 100, 0)
        report.append("")
        report.append("【特征追踪统计】")
        report.append(f"  追踪帧数:         {len(df_track)}")
        report.append(f"  候选特征 (均值):  {before.mean():.1f}")
        report.append(f"  RANSAC后 (均值):  {after_ransac.mean():.1f}")
        report.append(f"  总体通过率:       {overall.mean():.1f}%")
        report.append(f"  最低特征数:       {after_ransac.min()}")

    # 协方差统计
    if 'pose_cov_0' in df.columns:
        pos_cov_diag = [0, 7, 14]
        pos_std = []
        for idx in pos_cov_diag:
            col = f'pose_cov_{idx}'
            std = np.sqrt(np.abs(df[col].values))
            pos_std.append(std)
        report.append("")
        report.append("【协方差统计】 (位置标准差)")
        report.append(f"  σ_x 均值:         {pos_std[0].mean():.6f} m")
        report.append(f"  σ_y 均值:         {pos_std[1].mean():.6f} m")
        report.append(f"  σ_z 均值:         {pos_std[2].mean():.6f} m")

    report.append("")
    report.append("=" * 60)
    report.append("【生成的图表】")
    for f in sorted(os.listdir(data_dir)):
        if f.endswith('.png'):
            report.append(f"  - {f}")
    report.append("=" * 60)

    report_text = "\n".join(report)
    print(report_text)

    # 保存报告
    report_path = os.path.join(data_dir, 'analysis_report.txt')
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)
    print(f"\n  -> analysis_report.txt")

    # ======== 综合概览图 ========
    fig = plt.figure(figsize=(18, 12))

    # 3D 轨迹
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    sc = ax1.scatter(px, py, pz, c=t, cmap='viridis', s=1)
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Trajectory')

    # XY 俯视图
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.plot(px, py, 'b-', linewidth=0.5)
    ax2.plot(px[0], py[0], 'go', markersize=8)
    ax2.plot(px[-1], py[-1], 'r^', markersize=8)
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('XY Plane')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    # 速度
    ax3 = fig.add_subplot(2, 3, 3)
    if speed is not None:
        ax3.plot(t, speed, '#8e44ad', linewidth=0.8)
        ax3.fill_between(t, 0, speed, alpha=0.2, color='#8e44ad')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Speed (m/s)')
    ax3.set_title('Linear Speed')
    ax3.grid(True, alpha=0.3)

    # 欧拉角
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.plot(t, roll, 'r-', linewidth=0.8, label='Roll')
    ax4.plot(t, pitch, 'g-', linewidth=0.8, label='Pitch')
    ax4.plot(t, yaw, 'b-', linewidth=0.8, label='Yaw')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angle (°)')
    ax4.set_title('Euler Angles')
    ax4.legend(fontsize=9)
    ax4.grid(True, alpha=0.3)

    # 追踪
    ax5 = fig.add_subplot(2, 3, 5)
    if df_track is not None:
        t_tr = df_track['t'].values
        ax5.plot(t_tr, df_track['before_tracking'].values, 'b-', linewidth=0.8, label='Candidates')
        ax5.plot(t_tr, df_track['after_ransac'].values, 'r-', linewidth=0.8, label='After RANSAC')
        ax5.legend(fontsize=9)
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Feature Count')
    ax5.set_title('Feature Tracking')
    ax5.grid(True, alpha=0.3)

    # 协方差
    ax6 = fig.add_subplot(2, 3, 6)
    if 'pose_cov_0' in df.columns:
        for i, (idx, color, name) in enumerate(zip([0, 7, 14],
                                                    ['r', 'g', 'b'],
                                                    ['σ_x', 'σ_y', 'σ_z'])):
            col = f'pose_cov_{idx}'
            std = np.sqrt(np.abs(df[col].values))
            ax6.plot(t, std, color=color, linewidth=0.8, label=name)
        ax6.legend(fontsize=9)
    ax6.set_xlabel('Time (s)')
    ax6.set_ylabel('Std Dev (m)')
    ax6.set_title('Position Covariance')
    ax6.grid(True, alpha=0.3)

    fig.suptitle('MSCKF-VIO Analysis Overview', fontsize=16, y=1.01)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'overview.png'), dpi=150)
    plt.close(fig)
    print("  -> overview.png")


if __name__ == "__main__":
    main()
