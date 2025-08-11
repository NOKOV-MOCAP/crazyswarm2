import os
from setuptools import setup

package_name = 'nokov_swarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
        [os.path.join('launch', 'launch.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nokov',
    maintainer_email='lika.ji@nokov.com',
    description='Simple ground station for crazyswarm2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nokov_swarm_node = nokov_swarm.main:main',
        ],
    },
)

