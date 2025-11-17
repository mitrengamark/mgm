from setuptools import find_packages, setup

package_name = 'zh'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Későbbi bővítéskor ide jönnek: launch/, rviz/ stb.
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='ZH ament_python package skeleton.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_lis_node = zh.object_lis_node:main',
            # Példa később: 'zh_node = zh.zh_node:main'
        ],
    },
)
