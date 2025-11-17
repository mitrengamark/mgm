from setuptools import find_packages, setup

package_name = 'gyak10'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gyak10.launch.xml']),
        ('share/' + package_name + '/rviz', ['rviz/gyak10.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mgm',
    maintainer_email='doba.daniel@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ack_robot=gyak10.ack_robot:main",
            "pure_pursuit=gyak10.pure_pursuit:main"
        ],
    },
)
