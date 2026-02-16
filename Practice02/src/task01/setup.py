from setuptools import setup
import os
from glob import glob

package_name = 'task01'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/task01.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Task01 Receiver Node',
    license='TODO: License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = task01.receiver:main',
        ],
    },
)
