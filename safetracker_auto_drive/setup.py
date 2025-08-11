from setuptools import setup

package_name = 'safetracker_auto_drive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/safetracker_bringup.launch.py']),
        ('share/' + package_name + '/config', ['config/auto_drive.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Lane following + crosswalk stop (internal gating) + OpenCR bridge.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follow_node = safetracker_auto_drive.lane_follow_node:main',
            'crosswalk_node = safetracker_auto_drive.crosswalk_node:main',
            'serial_bridge_node = safetracker_auto_drive.serial_bridge_node:main',
            'lane_color_tuner = safetracker_auto_drive.lane_color_tuner:main',
        ],
    },
)
