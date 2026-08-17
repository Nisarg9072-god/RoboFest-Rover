from setuptools import setup

package_name = 'rover_behaviors'

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
    description='Behaviors: mission manager, parking, recovery',
    license='MIT',
    entry_points={
        'console_scripts': [
            'behavior_manager_node = rover_behaviors.behavior_manager_node:main',
            # mission_manager_node, parking_node, recovery_node: TBD
        ],
    },
)
