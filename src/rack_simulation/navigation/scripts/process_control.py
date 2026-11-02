"""Project-scoped process identities; never use substring-wide pkill."""
import json
import os
from pathlib import Path
import signal
import time

ROOT=Path(__file__).resolve().parent.parent


def identity(pid):
    try:
        stat=Path(f'/proc/{pid}/stat').read_text()
        fields=stat[stat.rfind(')')+2:].split()
        return dict(pid=int(pid),start=fields[19],state=fields[0],pgid=int(fields[2]),sid=int(fields[3]))
    except (OSError,ValueError,IndexError):return None


def same(saved):
    now=identity(saved['pid'])
    return now is not None and now['start']==saved['start'] and now['state']!='Z'


def terminate(saved,force=False):
    sig=signal.SIGKILL if force else signal.SIGTERM
    # If the leader was killed, descendants can still belong to its original session.
    if saved.get('group_owned') and saved['pgid']==saved['pid']:
        current=identity(saved['pid'])
        if current is not None and current['start']!=saved['start']:return
        members=group_members(saved)
        if members:os.killpg(saved['pgid'],sig)
    elif same(saved):os.kill(saved['pid'],sig)


def group_members(saved):
    if not saved.get('group_owned'):return [saved] if same(saved) else []
    result=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        current=identity(int(p.name))
        if (current and current['state']!='Z' and current['pgid']==saved['pgid'] and
                current['sid']==saved['pid'] and int(current['start'])>=int(saved['start'])):
            result.append(current)
    return result


def legacy_project_processes():
    result=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            if p.joinpath('cwd').resolve()!=ROOT:continue
            args=p.joinpath('cmdline').read_bytes().decode().split('\0')[:-1]
            if not args:continue
            exe=Path(args[0]).name
            is_python=exe.startswith('python') and any(
                a.endswith(('/mission_v3.py','/kinematic_driver.py','/lidar_localization.py','/map_alignment.py','/auto_next.py')) or
                a in ('scripts/mission_v3.py','scripts/kinematic_driver.py','scripts/lidar_localization.py','scripts/map_alignment.py','scripts/auto_next.py') for a in args[1:])
            is_ros=exe.startswith('python') and len(args)>1 and args[1].endswith('/ros2') and len(args)>2 and args[2] in ('run','launch')
            is_gz=(exe in ('gz','ruby','gz-sim-server','gz-sim-gui') and ('sim' in args or exe.startswith('gz-sim')))
            is_nav=('/opt/ros/jazzy/lib/nav2_' in args[0] or '/opt/ros/jazzy/lib/slam_toolbox/' in args[0] or
                    '/opt/ros/jazzy/lib/ros_gz_bridge/' in args[0] or '/opt/ros/jazzy/lib/tf2_ros/' in args[0])
            if is_python or is_ros or is_gz or is_nav:
                saved=identity(int(p.name))
                if saved:
                    saved['group_owned']=saved['pgid']==saved['pid']
                    result.append(saved)
        except (OSError,UnicodeError):continue
    return result


def stop_records(records):
    for saved in reversed(records):
        try:terminate(saved)
        except ProcessLookupError:pass
    deadline=time.monotonic()+5
    while time.monotonic()<deadline and any(group_members(s) for s in records):time.sleep(.1)
    for saved in reversed(records):
        try:terminate(saved,True)
        except ProcessLookupError:pass


def stop_project():
    registry=ROOT/'log/active_processes.json'
    if registry.exists():
        data=json.loads(registry.read_text())
        supervisor=data.get('supervisor')
        if supervisor and same(supervisor):
            terminate(supervisor)
            deadline=time.monotonic()+8
            while same(supervisor) and time.monotonic()<deadline:time.sleep(.1)
        stop_records(data.get('processes',[]))
    stop_records(legacy_project_processes())


if __name__=='__main__':
    stop_project();print('7503robot-main navigation processes stopped')
