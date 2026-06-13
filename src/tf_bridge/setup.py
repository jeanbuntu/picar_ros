from setuptools import find_packages, setup

package_name = 'tf_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jeano',
    maintainer_email='jeano@graymatter-robotics.com',
    description='Pi WebSocket → ROS2 TF + trajectory markers bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tf_bridge = tf_bridge.tf_bridge:main',
        ],
    },
)
