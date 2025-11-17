from setuptools import find_packages, setup
from glob import glob

package_name = 'proba2'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.xml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Feladat2 - Odometry feldolgozás, Marker, Header timer.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proba2_node = proba2.proba2_node:main',
        ],
    },
)
