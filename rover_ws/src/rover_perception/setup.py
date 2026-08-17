from setuptools import setup

package_name = 'rover_perception'

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
    description='Perception nodes: object detection, obstacle fusion, ArUco markers',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)
