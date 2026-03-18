#!/usr/bin/env python3
"""
公共数据加载模块
"""
import os
import sys
import numpy as np
import pandas as pd


def load_odom(data_dir):
    """加载 VIO 里程计 CSV，返回整理好的 DataFrame"""
    path = os.path.join(data_dir, "vio_odom.csv")
    if not os.path.isfile(path):
        print(f"[ERROR] 找不到文件: {path}")
        sys.exit(1)

    df = pd.read_csv(path)

    # 统一列名（去掉 field. 前缀）
    rename = {
        '%time': 'timestamp',
        'field.header.stamp': 'stamp',
        'field.pose.pose.position.x': 'px',
        'field.pose.pose.position.y': 'py',
        'field.pose.pose.position.z': 'pz',
        'field.pose.pose.orientation.x': 'qx',
        'field.pose.pose.orientation.y': 'qy',
        'field.pose.pose.orientation.z': 'qz',
        'field.pose.pose.orientation.w': 'qw',
        'field.twist.twist.linear.x': 'vx',
        'field.twist.twist.linear.y': 'vy',
        'field.twist.twist.linear.z': 'vz',
        'field.twist.twist.angular.x': 'wx',
        'field.twist.twist.angular.y': 'wy',
        'field.twist.twist.angular.z': 'wz',
    }
    df.rename(columns=rename, inplace=True)

    # 协方差列 (6x6 = 36 elements for pose, 36 for twist)
    for i in range(36):
        old_pose = f'field.pose.covariance{i}'
        old_twist = f'field.twist.covariance{i}'
        if old_pose in df.columns:
            df.rename(columns={old_pose: f'pose_cov_{i}'}, inplace=True)
        if old_twist in df.columns:
            df.rename(columns={old_twist: f'twist_cov_{i}'}, inplace=True)

    # 时间归零（秒）
    if 'timestamp' in df.columns:
        df['t'] = (df['timestamp'] - df['timestamp'].iloc[0]) * 1e-9
    elif 'stamp' in df.columns:
        df['t'] = (df['stamp'] - df['stamp'].iloc[0]) * 1e-9
    else:
        df['t'] = np.arange(len(df)) * 0.05  # fallback 20Hz

    return df


def load_tracking(data_dir):
    """加载特征追踪统计 CSV"""
    path = os.path.join(data_dir, "tracking_info.csv")
    if not os.path.isfile(path):
        print(f"[WARN] 找不到文件: {path}，跳过追踪分析")
        return None

    df = pd.read_csv(path)
    rename = {
        '%time': 'timestamp',
        'field.header.stamp': 'stamp',
        'field.before_tracking': 'before_tracking',
        'field.after_tracking': 'after_tracking',
        'field.after_matching': 'after_matching',
        'field.after_ransac': 'after_ransac',
    }
    df.rename(columns=rename, inplace=True)

    if 'timestamp' in df.columns:
        df['t'] = (df['timestamp'] - df['timestamp'].iloc[0]) * 1e-9

    return df


def quat_to_euler(qx, qy, qz, qw):
    """四元数转欧拉角 (roll, pitch, yaw)，单位：度"""
    # Roll (x)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def ensure_output_dir(data_dir):
    """确保输出目录存在"""
    os.makedirs(data_dir, exist_ok=True)
