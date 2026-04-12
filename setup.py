import os
from glob import glob

from setuptools import setup

package_name = 'ba_perception_pipeline'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Thomas',
    maintainer_email='noreply@example.org',
    description='ROS2 perception pipeline for the BA robot arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_pipeline_node = ba_perception_pipeline.perception_pipeline_node:main',
        ],
    },
)
