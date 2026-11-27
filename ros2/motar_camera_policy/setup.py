from glob import glob
from setuptools import find_packages, setup


package_name = 'motar_camera_policy'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MOTAR contributors',
    maintainer_email='motar@example.invalid',
    description='Explicit-source ROS2 RGB-D camera-policy proposals and metadata checks.',
    license='BSD-3-Clause',
    entry_points={'console_scripts': ['camera_policy = motar_camera_policy.entry:main',
                                     'camera_px4_gateway = motar_camera_policy.entry:gateway_main']},
)
