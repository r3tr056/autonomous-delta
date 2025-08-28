from setuptools import find_packages, setup

package_name = 'delta_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='retro',
    maintainer_email='dangerankur56@gmail.com',
    description='Delta robot inverse kinematics controller and simulation node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delta_ik = delta_robot_control.delta_ik:main',
            'delta_ik.py = delta_robot_control.delta_ik:main',
            'delta_robot_sim = delta_robot_control.delta_robot_sim:main',
            'delta_robot_sim.py = delta_robot_control.delta_robot_sim:main',
            # Note: delta_constraints referenced in constrained launch but not implemented
            # 'delta_constraints = delta_robot_control.delta_constraints:main',
        ],
    },
)
