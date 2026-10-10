#!/usr/bin/env python3
"""离线汇总 CSV，不启动 ROS。默认读取本项目最新联合验证运行。"""
import argparse
import csv
import math
from pathlib import Path

def main():
    parser=argparse.ArgumentParser();parser.add_argument('file',nargs='?');args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    files=list((root/'navigation/log').glob('run_*/hybrid_metrics.csv'))
    filename=Path(args.file) if args.file else max(files,key=lambda p:p.stat().st_mtime) if files else None
    if filename is None:raise SystemExit('还没有联合验证 CSV；先由操作者运行 run_rack_hybrid.sh')
    with filename.open(encoding='utf-8') as f:rows=list(csv.DictReader(f))
    print('CSV:',filename,'\n配对样本:',len(rows))
    if not rows:raise SystemExit('没有有效时间配对样本；检查真值/odom 时间戳及 evaluation/metrics')
    for label,key in [('定位误差(m)','position_error_m'),('航向误差(rad)','yaw_error_rad'),
                      ('跟踪期间路径偏差(m)','cross_track_error_m')]:
        values=[float(r[key]) for r in rows if r[key]!='']
        if values:
            print(label,'RMSE=',math.sqrt(sum(v*v for v in values)/len(values)),'MAX=',max(values))
        else:print(label,'无有效跟踪样本')
    for key in ('cmd_vx','cmd_vy','cmd_wz'):print(key,'发布绝对值最大=',max(abs(float(r[key])) for r in rows))
    print('观察到的状态:',list(dict.fromkeys(r['state'] for r in rows if r['state'])))
    print('仅为本次 Gazebo 软件执行结果，不是实物验收；Dock 与导航速度统计包含不同控制阶段。')

if __name__=='__main__':main()
