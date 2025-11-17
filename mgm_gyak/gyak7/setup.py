from setuptools import find_packages, setup

package_name = 'gyak7'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gyak7.launch.xml']),
        ('share/' + package_name + '/rviz', ['rviz/gyak7.rviz'])
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
            "diff_robot=gyak7.diff_robot:main",
            "control=gyak7.control:main"
        ],
    },
)
