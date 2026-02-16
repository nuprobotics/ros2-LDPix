from setuptools import setup

package_name = 'task03'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
         ['launch/task03.launch.py']),
        ('share/' + package_name + '/config',
         ['config/task03.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Task03 Trigger proxy node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trigger_proxy = task03.trigger_proxy:main',
        ],
    },
)
