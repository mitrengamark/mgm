from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'proba2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch fájlok (gyak10/setup.py mintájára)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xml')),
        # RViz config (gyak12/setup.py mintájára)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mark',
    maintainer_email='mitrengamark@edu.bme.hu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                    'node1 = proba2.node1:main',
        ],
    },
)
