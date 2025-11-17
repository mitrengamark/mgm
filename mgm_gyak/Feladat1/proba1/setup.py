from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'proba1'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Feladat1 - proba1 package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proba1_node = proba1.proba1_node:main',
        ],
    },
)
