from setuptools import setup
import os
from glob import glob

package_name = 'agv_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aorus-ubun',
    maintainer_email='oatdev54@gmail.com',
    description='AGV bringup launch files',
    license='Apache-2.0',

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],

    entry_points={
        'console_scripts': [],
    },
)