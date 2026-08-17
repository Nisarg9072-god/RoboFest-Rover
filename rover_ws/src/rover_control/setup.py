from setuptools import setup, find_packages

package_name = 'rover_control'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='team@robofest.local',
    description='Motor control: Arduino interface, motion controller',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arduino_interface_node = rover_control.arduino_interface_node:main',
            # motion_controller_node: TBD
        ],
    },
)
