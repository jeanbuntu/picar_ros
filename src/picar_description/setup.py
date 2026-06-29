from setuptools import setup
import os
from glob import glob

package_name = 'picar_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeano',
    maintainer_email='jeano@graymatter-robotics.com',
    description='PiCar-X URDF for live tracking visualization in RViz2.',
    license='MIT',
    entry_points={},
)
