from setuptools import setup

package_name = 'anatolian_rover_mvp'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mvp_sim.launch.py']),
        ('share/' + package_name + '/config', ['config/sim_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='earendilrover',
    maintainer_email='roverearendil@gmail.com',
    description='Minimal GPS-based autonomous rover nodes for Anatolian Rover.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager_node = anatolian_rover_mvp.mission_manager_node:main',
            'sim_gps_data_node = anatolian_rover_mvp.sim_gps_data_node:main',
            'sim_imu9250_node = anatolian_rover_mvp.sim_imu9250_node:main',
            'heading_node = anatolian_rover_mvp.heading_node:main',
            'navigation_node = anatolian_rover_mvp.navigation_node:main',
            'sim_motor_control_node = anatolian_rover_mvp.sim_motor_control_node:main',
            'rover_monitor_node = anatolian_rover_mvp.rover_monitor_node:main',
        ],
    },
)
