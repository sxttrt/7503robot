#!/usr/bin/env python3
"""Detached supervisor for this project only. Executed by the user's run.sh."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
from process_control import ROOT, identity, same, legacy_project_processes, stop_records


class Stack:
    def __init__(self,run):
        self.run=run;self.children=[];self.records=[];self.stop=False;self.ready=False;self.completed=False;self.gui_closed=False;self.mission_failed=False
        self.registry=ROOT/'log/active_processes.json'
    def say(self,msg):print(f'[{time.strftime("%H:%M:%S")}] {msg}',flush=True)
    def save(self):
        supervisor=identity(os.getpid());supervisor['group_owned']=True
        data=dict(supervisor=supervisor,processes=self.records,run=str(self.run))
        temp=self.registry.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2));temp.replace(self.registry)
    def start(self,name,args):
        log=self.run/f'{name}.log';handle=log.open('w')
        p=subprocess.Popen(args,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=handle,
                           stderr=subprocess.STDOUT,start_new_session=True)
        handle.close();saved=identity(p.pid)
        if saved is None:raise RuntimeError(f'{name} exited during launch')
        saved['group_owned']=True;saved['name']=name
        self.children.append((name,p,log));self.records.append(saved);self.save()
        self.say(f'{name} PID={p.pid}')
    def mark_mission_failed(self,detail):
        if self.mission_failed:return
        self.mission_failed=True
        self.run.joinpath('status').write_text('MISSION_FAILED\n')
        self.say(f'任务故障；保留仿真供检查。查看 mission.log 的原因，'
                 f'使用本次启动入口对应的停止脚本停止后再重启。\n{detail}')

    def alive(self):
        if self.stop:raise KeyboardInterrupt
        for name,p,log in self.children:
            code=p.poll()
            if code is None:continue
            if name=='gz_gui':
                if not self.gui_closed:
                    self.gui_closed=True;self.say(f'GUI 已退出 code={code}；后台仿真和任务继续')
                continue
            if name=='mission' and code==0 and 'state=DONE' in log.read_text(errors='replace'):
                if not self.completed:
                    self.completed=True;self.run.joinpath('status').write_text('DONE\n')
                    self.say('任务全部完成；仿真保留，停止请执行 bash scripts/stop.sh')
                continue
            tail=log.read_text(errors='replace')[-2500:]
            if name=='mission' and self.ready:
                self.mark_mission_failed(f'exit code={code}\n{tail}')
                continue
            raise RuntimeError(f'{name} exited code={code}\n{tail}')
        return True
    def wait_for(self,predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if not self.alive():raise RuntimeError('mission completed before startup ready')
            if predicate():return
            time.sleep(.2)
        raise RuntimeError(f'{label} readiness timed out ({timeout}s)')
    def cleanup(self):
        stop_records(self.records)
        if self.registry.exists():
            try:
                data=json.loads(self.registry.read_text())
                if data.get('supervisor',{}).get('pid')==os.getpid():self.registry.unlink()
            except (OSError,ValueError):pass


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--run-dir',required=True,type=Path)
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--framework',type=Path);parser.add_argument('--hybrid',action='store_true');args=parser.parse_args()
    if args.hybrid and not args.framework:parser.error('--hybrid requires --framework')
    ROOT.joinpath('log').mkdir(exist_ok=True)
    lock=ROOT.joinpath('log/stack.lock').open('a')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        print('已有本项目导航仿真运行；先由你执行 bash scripts/stop.sh',flush=True);return 2
    stack=Stack(args.run_dir)
    def request_stop(signum,_frame):
        stack.say(f'supervisor signal={signal.Signals(signum).name}');stack.stop=True
    for sig in (signal.SIGINT,signal.SIGTERM):signal.signal(sig,request_stop)
    try:
        # Migrate pre-supervisor runs using exact cwd and known executables only.
        previous=legacy_project_processes()
        if previous:stack.say('停止本项目旧实例');stop_records(previous)
        stack.save();stack.say('生成世界与 QR')
        with (stack.run/'build.log').open('w') as log:
            subprocess.run([sys.executable,'tools/build_world.py'],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
        stack.say('[1/6] Gazebo server');stack.start('gz_srv',['gz','sim','-s','-r','worlds/rack_world.sdf'])
        # No synthetic ROS probes: driver and mission themselves validate live data.
        time.sleep(2);stack.alive()
        if not args.headless:stack.say('[2/6] Gazebo GUI');stack.start('gz_gui',['gz','sim','-g'])
        stack.say('[3/6] bridge + TF + driver')
        stack.start('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','--ros-args','-p',f'config_file:={ROOT}/config/bridge.yaml'])
        if args.hybrid:
            os.environ['HYBRID_EVALUATION_FILE']=str(stack.run/'hybrid_metrics.csv')
            stack.start('driver',[sys.executable,'scripts/kinematic_driver.py','--ros-args',
                                 '-p','use_sim_time:=true','-p','allow_scan_rotation:=true'])
            stack.say('[4/6-5/6] 实机定位/导航/跟踪和 costmap 由联合 launch 统一启动')
        else:
            from scene import LIDAR_Z
            stack.start('tf',['ros2','run','tf2_ros','static_transform_publisher','--x','0','--y','0','--z',str(LIDAR_Z),
                              '--roll','0','--pitch','0','--yaw','0','--frame-id','base_link','--child-frame-id','laser'])
            driver_args=[sys.executable,'scripts/kinematic_driver.py','--ros-args','-p','use_sim_time:=true']
            if args.framework:driver_args+=['-p','allow_scan_rotation:=true']
            stack.start('lidar',[sys.executable,'scripts/lidar_localization.py','--ros-args','-p','use_sim_time:=true'])
            stack.start('driver',driver_args)
            stack.start('map_alignment',[sys.executable,'scripts/map_alignment.py','--ros-args','-p','use_sim_time:=true'])
            stack.say('[4/6] SLAM')
            stack.start('slam',['ros2','launch','slam_toolbox','online_async_launch.py',f'slam_params_file:={ROOT}/config/slam_toolbox.yaml','use_sim_time:=true'])
            stack.say('[5/6] Nav2 costmaps')
            stack.start('nav2',['ros2','launch','launch/table_nav.launch.py',f'params_file:={ROOT}/config/nav2_params.yaml',
                              'use_sim_time:=true','autostart:=true'])
        stack.say('[6/6] framework (自动动作，等待框架QR结果)' if args.framework else '[6/6] legacy mission (一次触发一个目标，到位自动动作)')
        if args.framework:
            stack.say('使用 7503 mission_manager + framework_execution；不启动旧任务循环')
            stack.start('mission',['ros2','launch','robot_bringup','rack_hybrid.launch.py' if args.hybrid else 'rack_simulation.launch.py'])
        else:stack.start('mission',[sys.executable,'scripts/mission_v3.py','--ros-args','-p','use_sim_time:=true'])
        mission_log=stack.run/'mission.log'
        startup_offset=0
        def mission_ready():
            nonlocal startup_offset
            text=mission_log.read_text(errors='replace')
            for line in text[startup_offset:].splitlines():
                if 'STARTUP_WAIT:' in line or 'FRAMEWORK_STARTUP_FAILED:' in line:stack.say(line)
            startup_offset=len(text)
            if args.framework:
                return ('HYBRID_ADAPTER_READY:' if args.hybrid else 'FRAMEWORK_READY:') in text and 'IDLE → NAV_START' in text
            return '等待开关:' in text
        stack.wait_for(mission_ready,120,'mission/map/TF/framework')
        stack.ready=True;stack.run.joinpath('status').write_text('READY\n');stack.say('就绪：数据、地图和接口已确认；框架按识别结果自动推进' if args.framework else '就绪：已确认数据、地图、足迹和第一条路径')
        while stack.alive():
            mission_text=mission_log.read_text(errors='replace')
            if args.framework and 'FRAMEWORK_FINISHED:' in mission_text:
                stack.run.joinpath('status').write_text('DONE\n')
                stack.say('终点确认完成，结束本次框架及仿真进程')
                return 0
            if 'FAILED' in mission_text or 'Traceback' in mission_text or '→ FAULT' in mission_text or 'process has died' in mission_text:
                stack.mark_mission_failed(mission_text[-2500:])
            nav_text=stack.run.joinpath('nav2.log').read_text(errors='replace') if not args.hybrid else ''
            slam_text=stack.run.joinpath('slam.log').read_text(errors='replace') if not args.hybrid else ''
            if 'process has died' in nav_text or 'process has died' in slam_text:raise RuntimeError('Nav2/SLAM child died; inspect component logs')
            time.sleep(.5)
        stack.run.joinpath('status').write_text('DONE\n');return 0
    except KeyboardInterrupt:
        stack.run.joinpath('status').write_text('STOPPED\n');return 0
    except Exception as e:
        stack.say(f'FATAL: {e}');traceback.print_exc()
        stack.run.joinpath('status').write_text('FAILED\n');return 1
    finally:stack.cleanup();lock.close()


if __name__=='__main__':sys.exit(main())
