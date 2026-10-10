"""读取并验证中文配置；实机缺项必须报错，不能退回模拟数据。"""

import math
from pathlib import Path
import re

import yaml


def read_yaml(path):
    """使用 UTF-8 读取配置，顶层必须是字典。"""
    with Path(path).open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f'配置必须是字典：{path}')
    return data


def number(value, name, minimum=0.0, allow_zero=False):
    """拒绝布尔值、无穷值和负数，避免错误配置绕过超时。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} 必须是数字')
    if not math.isfinite(value) or (value < minimum if allow_zero else value <= minimum):
        raise ValueError(f'{name} 数值不合法：{value}')


def validate(config, mode):
    """校验固定顺序、动作限时和本队二维码；导航坐标不属于任务配置。"""
    if mode not in ('simulation', 'robot'):
        raise ValueError('运行模式只能是 simulation 或 robot')
    if not isinstance(config, dict) or any(not isinstance(config.get(key), dict)
                                          for key in ('mission', 'interfaces', 'targets')):
        raise ValueError('mission、interfaces、targets 必须是配置字典')
    mission, interfaces, targets = (config[key] for key in ('mission', 'interfaces', 'targets'))
    required = ('game_duration_sec', 'stop_timeout_sec', 'health_timeout_sec',
                'qr_max_age_sec', 'qr_future_tolerance_sec', 'tick_period_sec',
                'qr_confirm_frames', 'timeouts', 'retries')
    if any(key not in mission for key in required):
        raise ValueError('任务配置缺少必需字段：' + '、'.join(key for key in required if key not in mission))
    if not isinstance(mission['timeouts'], dict) or not isinstance(mission['retries'], dict):
        raise ValueError('timeouts 和 retries 必须是字典')
    if mission.get('team_number') != 2:
        raise ValueError('本工程当前固定为第二队，team_number 必须为 2')
    if mission.get('rack_order') != ['A', 'B', 'C', 'D']:
        raise ValueError('货架顺序必须严格为 A、B、C、D，禁止自动重排或跳过')
    for key in ('game_duration_sec', 'stop_timeout_sec', 'health_timeout_sec',
                'qr_max_age_sec', 'qr_future_tolerance_sec', 'tick_period_sec'):
        number(mission[key], key)
    frames = mission['qr_confirm_frames']
    if type(frames) is not int or frames < 1:
        raise ValueError('qr_confirm_frames 必须是正整数')
    for state in ('WAIT_START', 'NAV_RACK', 'WAIT_RACK_QR', 'DOCK_ENTER', 'LIFT_UP',
                  'NAV_END', 'WAIT_END_QR', 'LIFT_DOWN', 'DOCK_EXIT'):
        if state not in mission['timeouts']:
            raise ValueError(f'缺少步骤时限：timeouts.{state}')
        number(mission['timeouts'][state], f'timeouts.{state}')
        attempts = mission['retries'].get(state, 0)
        if type(attempts) is not int or attempts < 0:
            raise ValueError(f'retries.{state} 必须是非负整数')
    if type(mission.get('confirm_end_qr')) is not bool:
        raise ValueError('confirm_end_qr 必须是 true 或 false')
    if mode == 'robot' and mission['game_duration_sec'] != 180:
        raise ValueError('实机比赛时间必须为课程规定的 180 秒')
    for state in ('DOCK_ENTER', 'LIFT_UP', 'NAV_END', 'LIFT_DOWN', 'DOCK_EXIT'):
        if mission['retries'].get(state, 0) != 0:
            raise ValueError(f'第一版禁止 {state} 自动重试，避免位置或携货状态不明时继续动作')
    if mission['tick_period_sec'] >= min(*mission['timeouts'].values(),
                                       mission['stop_timeout_sec'], mission['health_timeout_sec']):
        raise ValueError('调度周期必须短于步骤、停止和健康状态的超时时间')
    if interfaces.get('lift_completion') != 'response_means_completed':
        raise ValueError('第一版升降响应必须代表完成；仅接受命令的服务需要先改写适配器')
    for key in ('scan_route','qr_timeout_advance','shutdown_on_finish'):
        if type(mission.get(key,False)) is not bool:raise ValueError(key+' 必须是布尔值')
    if mode=='robot' and mission.get('qr_timeout_advance',False):
        raise ValueError('实机禁止二维码超时放行')
    if mission.get('scan_route',False):
        for state in ('NAV_START','NAV_FINAL','WAIT_FINAL_QR'):
            if state not in mission['timeouts']:raise ValueError('缺少扫描步骤时限：'+state)
            number(mission['timeouts'][state],state)
    used_names = {'mission/arm': '主程序启用服务', 'mission/stop': '主程序人工停止服务'}
    for key in ('navigation_action', 'qr_topic', 'docking_action', 'lift_service',
                'lift_state_topic', 'health_topic', 'stop_service', 'mission_status_topic',
                *[k for k in ('pose_navigation_action','tracking_action','navigation_stop_service',
                              'navigation_status_topic','base_feedback_topic') if k in interfaces]):
        name = interfaces.get(key)
        if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*', name):
            raise ValueError(f'{key} 必须是合法相对名称，禁止空格、重复斜杠和绝对路径')
        if name in used_names:
            raise ValueError(f'{key} 与 {used_names[name]} 名称不能相同：{name}，避免串线或调用自身')
        used_names[name] = key
    if mode == 'robot' and targets.get('simulation_only') is not False:
        raise ValueError('实机模式禁止使用模拟目标配置')
    if mode == 'simulation' and targets.get('simulation_only') is not True:
        raise ValueError('模拟模式必须显式使用模拟目标配置')
    missing = []
    for rack in mission['rack_order']:
        item = targets.get('racks', {}).get(rack, {})
        raw = item.get('qr')
        if not isinstance(raw, str) or not re.fullmatch(rf'RACK{rack}_[A-Za-z0-9]+', raw):
            missing.append(f'racks.{rack}.qr（第二队完整二维码）')
        elif mode == 'robot' and raw.endswith('_SIMTEAM2'):
            missing.append(f'racks.{rack}.qr（禁止使用模拟后缀）')
    if missing:
        raise ValueError('以下配置尚未填写或无效：\n  ' + '\n  '.join(missing))
    config['mode'] = mode
    return config


def load_config(mission_file, interfaces_file, targets_file, mode):
    """三个配置来源必须明确指定，禁止自动猜测真实目标数据。"""
    return validate({
        'mission': read_yaml(mission_file),
        'interfaces': read_yaml(interfaces_file),
        'targets': read_yaml(targets_file),
    }, mode)
