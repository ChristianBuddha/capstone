from setuptools import find_packages, setup

package_name = 'nexus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nexus_full_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hs',
    maintainer_email='quietquiet35@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'control_nexus = nexus.control_nexus:main',
        'odom_serial_node = nexus.odom_serial_node:main',
        'wheel_encoder = nexus.wheel_encoder:main',
        'odom_monitor = nexus.odom_monitor:main',
        'test = nexus.test:main',
        'joy_to_cmd_vel = nexus.joy_to_cmd_vel:main',
        ],
    },
)
