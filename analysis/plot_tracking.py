#!/usr/bin/env python3
"""
特征追踪统计分析
- 各阶段特征数量时间序列
- 追踪成功率
- 特征数量分布直方图
- 滑动窗口统计
"""
import sys
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from data_loader import load_tracking, ensure_output_dir


def main():
    data_dir = sys.argv[1]
    df = load_tracking(data_dir)
    if df is None:
        return
    ensure_output_dir(data_dir)

    t = df['t'].values
    before = df['before_tracking'].values
    after_track = df['after_tracking'].values
    after_match = df['after_matching'].values
    after_ransac = df['after_ransac'].values

    # ======== 图1: 特征数量时间序列 ========
    fig, ax = plt.subplots(figsize=(14, 6))

    ax.plot(t, before, '-', color='#3498db', linewidth=0.8, alpha=0.8, label='Before Tracking')
    ax.plot(t, after_track, '-', color='#2ecc71', linewidth=0.8, alpha=0.8, label='After Tracking')
    ax.plot(t, after_match, '-', color='#e67e22', linewidth=0.8, alpha=0.8, label='After Matching')
    ax.plot(t, after_ransac, '-', color='#e74c3c', linewidth=1.0, label='After RANSAC')

    ax.fill_between(t, after_ransac, before, alpha=0.1, color='gray')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Feature Count', fontsize=12)
    ax.set_title('MSCKF-VIO Feature Tracking Pipeline', fontsize=14)
    ax.legend(fontsize=11, loc='upper right')
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'tracking_features.png'), dpi=150)
    plt.close(fig)
    print("  -> tracking_features.png")

    # ======== 图2: 追踪成功率 ========
    # 追踪率 = after_tracking / before_tracking
    # RANSAC 内点率 = after_ransac / after_matching
    track_rate = np.where(before > 0, after_track / before * 100, 0)
    ransac_rate = np.where(after_match > 0, after_ransac / after_match * 100, 0)
    overall_rate = np.where(before > 0, after_ransac / before * 100, 0)

    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    axes[0].plot(t, track_rate, '#2ecc71', linewidth=0.8)
    axes[0].axhline(y=np.mean(track_rate), color='red', linestyle='--', alpha=0.5)
    axes[0].set_ylabel('Tracking Rate (%)')
    axes[0].set_title(f'Tracking Rate (Mean: {np.mean(track_rate):.1f}%)', fontsize=12)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim(0, 105)

    axes[1].plot(t, ransac_rate, '#e67e22', linewidth=0.8)
    axes[1].axhline(y=np.mean(ransac_rate), color='red', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('RANSAC Inlier Rate (%)')
    axes[1].set_title(f'RANSAC Inlier Rate (Mean: {np.mean(ransac_rate):.1f}%)', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylim(0, 105)

    axes[2].plot(t, overall_rate, '#e74c3c', linewidth=0.8)
    axes[2].axhline(y=np.mean(overall_rate), color='red', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Overall Rate (%)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title(f'Overall Pass Rate (Mean: {np.mean(overall_rate):.1f}%)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_ylim(0, 105)

    fig.suptitle('MSCKF-VIO Feature Tracking Rates', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'tracking_rates.png'), dpi=150)
    plt.close(fig)
    print("  -> tracking_rates.png")

    # ======== 图3: 特征数量分布 ========
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    data_list = [
        (before, 'Before Tracking', '#3498db'),
        (after_track, 'After Tracking', '#2ecc71'),
        (after_match, 'After Matching', '#e67e22'),
        (after_ransac, 'After RANSAC', '#e74c3c'),
    ]

    for ax, (data, name, color) in zip(axes.flat, data_list):
        ax.hist(data, bins=30, color=color, alpha=0.7, edgecolor='white')
        ax.axvline(x=np.mean(data), color='red', linestyle='--',
                   label=f'Mean: {np.mean(data):.1f}')
        ax.axvline(x=np.median(data), color='blue', linestyle=':',
                   label=f'Median: {np.median(data):.1f}')
        ax.set_xlabel('Feature Count')
        ax.set_ylabel('Frequency')
        ax.set_title(name)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)

    fig.suptitle('Feature Count Distribution', fontsize=14)
    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'tracking_histogram.png'), dpi=150)
    plt.close(fig)
    print("  -> tracking_histogram.png")

    # ======== 图4: 丢失特征数量 ========
    lost_tracking = before - after_track
    lost_matching = after_track - after_match
    lost_ransac = after_match - after_ransac

    fig, ax = plt.subplots(figsize=(14, 6))
    ax.stackplot(t, lost_tracking, lost_matching, lost_ransac,
                 labels=['Lost in Tracking', 'Lost in Matching', 'Lost in RANSAC'],
                 colors=['#3498db', '#e67e22', '#e74c3c'], alpha=0.7)
    ax.plot(t, before, 'k-', linewidth=0.8, alpha=0.5, label='Total Candidates')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Feature Count', fontsize=12)
    ax.set_title('Feature Loss at Each Stage', fontsize=14)
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(data_dir, 'tracking_loss.png'), dpi=150)
    plt.close(fig)
    print("  -> tracking_loss.png")


if __name__ == "__main__":
    main()
