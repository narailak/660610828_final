from setuptools import find_packages, setup

package_name = 'agv_motion_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aorus-ubun',
    maintainer_email='oatdev54@gmail.com',
    description='AGV motion manager',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'motion_manager = agv_motion_manager.motion_manager:main',
        ],
    },
)