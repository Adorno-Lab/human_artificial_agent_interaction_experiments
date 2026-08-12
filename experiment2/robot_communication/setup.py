from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_communication'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, package_name, 'facial_expressions'), glob(os.path.join(package_name, 'facial_expressions', '*.[pxy]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anacampos',
    maintainer_email='anachristinaac@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_communication_node = robot_communication.robot_communication_node:main'
        ],
    },
)
