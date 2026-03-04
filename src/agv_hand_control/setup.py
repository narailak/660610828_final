from setuptools import find_packages, setup

package_name = 'agv_hand_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'lib/python3.12/site-packages/{package_name}', 
            ['agv_hand_control/hand_landmarker.task']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aorus-ubun',
    maintainer_email='oatdev54@gmail.com',
    description='AGV control using MediaPipe hand gesture',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mediapipe_node = agv_hand_control.mediapipe_node:main',
        ],
    },
)
