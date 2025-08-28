from setuptools import find_packages, setup
import os


package_name = 'delta_robot_hardware'

def collect_models_data(root_dir: str):
    data = []
    if not os.path.isdir(root_dir):
        return data
    for dirpath, _, filenames in os.walk(root_dir):
        files = [os.path.join(dirpath, f) for f in filenames]
        if not files:
            continue
        install_dir = os.path.join('share', package_name, dirpath)
        data.append((install_dir, files))
    return data

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=(
        [
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ]
        + collect_models_data('models')
    ),
    install_requires=[
        'setuptools',
        'ncnn>=1.0.0',
        'psutil>=5.0.0',
        'numpy>=1.19.0',
    ],
    zip_safe=True,
    maintainer='retro',
    maintainer_email='dangerankur56@gmail.com',
    description='Hardware interface nodes for delta robot: camera, detector, ArUco tracker, and PS4 teleop',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = delta_robot_hardware.camera_node:main',
            'camera_node.py = delta_robot_hardware.camera_node:main',
            'detector_node = delta_robot_hardware.detector_node:main',
            'detector_node.py = delta_robot_hardware.detector_node:main',
            'ee_aruco_tracker = delta_robot_hardware.ee_aruco_tracker:main',
            'ee_aruco_tracker.py = delta_robot_hardware.ee_aruco_tracker:main',
            'teleop_node = delta_robot_hardware.teleop_node:main',
            'teleop_node.py = delta_robot_hardware.teleop_node:main',
        ],
    },
)
