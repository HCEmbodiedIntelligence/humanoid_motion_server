from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_motion_server'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', [f for f in glob('launch/*') if os.path.isfile(f)]),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maple',
    maintainer_email='mapleswfx@gmail.com',
    description='Pinocchio-powered Humanoid Motion Control Server',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'humanoid_motion_control_node = humanoid_motion_server.motion_node:main',
        ],
    },
)
