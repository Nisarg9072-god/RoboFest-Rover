from setuptools import setup

package_name = 'rover_manipulation'

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
    description='Robotic arm and gripper manipulation for movable obstacles',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_controller_node = rover_manipulation.arm_controller_node:main',
            'gripper_controller_node = rover_manipulation.gripper_controller_node:main',
            'manipulation_manager_node = rover_manipulation.manipulation_manager_node:main',
        ],
    },
)
