from setuptools import find_packages, setup
setup(name='robot_navigation', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages',['resource/robot_navigation']),
                  ('share/robot_navigation',['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='DASE7503 第二队', maintainer_email='team2@example.invalid',
      description='实机定位和导航，不启动仿真', license='Apache-2.0',
      entry_points={'console_scripts':[
          'lidar_localization = robot_navigation.localization:main',
          'navigation_server = robot_navigation.navigation:main',
          'path_tracker = robot_navigation.path_tracker:main',
          'hybrid_evaluator = robot_navigation.hybrid_evaluator:main',
          'chassis_test = robot_navigation.chassis_test:main']})
