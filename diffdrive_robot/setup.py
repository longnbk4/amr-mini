from setuptools import find_packages, setup

package_name = 'diffdrive_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='longnb',
    maintainer_email='longnb@example.com',
    description='Teleop ban phim + node cau noi diff-drive (ROS2 <-> ESP32 qua UART)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard_node = diffdrive_robot.teleop_keyboard_node:main',
            'diffdrive_node = diffdrive_robot.diffdrive_node:main',
        ],
    },
)
