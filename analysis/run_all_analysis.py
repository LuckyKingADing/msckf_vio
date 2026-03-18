#!/usr/bin/env python3
"""
MSCKF-VIO 结果分析工具包
========================
自动加载 CSV 数据并生成全面的分析图表。

用法:
    python3 run_all_analysis.py <数据目录>

示例:
python3 run_all_analysis.py ../datasets_output/machine_hall/MH_01_easy/20260318

MH_01_easy 
MH_02_easy
MH_03_medium
MH_04_difficult
MH_05_difficult
    
"""

import sys
import os
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def main():
    if len(sys.argv) < 2:
        print("用法: python3 run_all_analysis.py <数据目录路径>")
        print("示例: python3 run_all_analysis.py ../datasets_output/20260318-14")
        sys.exit(1)

    data_dir = os.path.abspath(sys.argv[1])
    if not os.path.isdir(data_dir):
        print(f"[ERROR] 目录不存在: {data_dir}")
        sys.exit(1)

    scripts = [
        ("plot_trajectory.py",    "3D/2D 轨迹可视化"),
        ("plot_position.py",      "位置 XYZ 时间序列"),
        ("plot_orientation.py",   "姿态四元数 & 欧拉角"),
        ("plot_velocity.py",      "线速度 & 角速度"),
        ("plot_covariance.py",    "位姿协方差分析"),
        ("plot_tracking.py",      "特征追踪统计"),
        ("generate_report.py",    "汇总报告"),
    ]

    print("=" * 60)
    print("MSCKF-VIO 结果分析工具")
    print(f"数据目录: {data_dir}")
    print("=" * 60)

    success = 0
    failed = 0
    for script, desc in scripts:
        script_path = os.path.join(SCRIPT_DIR, script)
        if not os.path.isfile(script_path):
            print(f"  [SKIP] {script} 不存在")
            continue
        print(f"\n>>> [{desc}] 运行 {script} ...")
        ret = subprocess.run(
            [sys.executable, script_path, data_dir],
            cwd=SCRIPT_DIR
        )
        if ret.returncode == 0:
            print(f"  [OK] {desc} 完成")
            success += 1
        else:
            print(f"  [FAIL] {desc} 失败 (退出码: {ret.returncode})")
            failed += 1

    print("\n" + "=" * 60)
    print(f"全部完成: {success} 成功, {failed} 失败")
    print(f"结果保存在: {data_dir}/")
    print("=" * 60)


if __name__ == "__main__":
    main()
