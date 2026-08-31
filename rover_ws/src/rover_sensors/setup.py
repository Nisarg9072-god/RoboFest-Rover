from setuptools import setup

package_name = 'rover_sensors'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='team@robofest.local',
    description='Sensor driver nodes: LiDAR, camera, IMU, ToF, ultrasonic',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tof_node = rover_sensors.tof_node:main',
            'lidar_node = rover_sensors.lidar_node:main',
            'camera_node = rover_sensors.camera_node:main',
            'imu_node = rover_sensors.imu_node:main',
            'ultrasonic_node = rover_sensors.ultrasonic_node:main',
        ],
    },
)
