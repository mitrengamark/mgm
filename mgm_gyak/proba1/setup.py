from setuptools import find_packages, setup

package_name = 'proba1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/proba1.launch.xml']),
        ('share/' + package_name + '/rviz', ['rviz/proba1.rviz']),
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
            'node1=proba1.node1:main',
        ],
    },
)
