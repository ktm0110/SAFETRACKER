from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'SAFETRACKER'

data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.[pxy][yma]'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
    ]

def package_files(data_file, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for path, _, filenames in os.walk(directory):
            install_path = os.path.join('share', package_name, path)
            file_paths = [os.path.join(path, filename) for filename in filenames]
            if install_path in paths_dict:
                paths_dict[install_path].extend(file_paths)
            else:
                paths_dict[install_path] = file_paths
    for install_path, files in paths_dict.items():
        data_file.append((install_path, files))
    
    return data_file

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['models/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_viewer = rover_line_follower.command_viewer:main'
        ],
    },
)
