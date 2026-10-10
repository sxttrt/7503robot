from setuptools import find_packages, setup

# 此文件负责让 ROS 找到两个可执行节点；业务流程放在状态机文件中。
setup(
    name='mission_manager',
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/mission_manager']),
        ('share/mission_manager', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DASE7503 第二队',
    maintainer_email='team2@example.invalid',
    description='任务状态机、接口与模拟模块',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission_node = mission_manager.mission_node:main',
        'mock_modules = mission_manager.mock_modules:main',
        'framework_execution = mission_manager.framework_execution:main',
        'hybrid_execution = mission_manager.hybrid_execution:main',
    ]},
)
