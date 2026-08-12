from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'screen_applications'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, package_name, 'images'), glob(os.path.join(package_name, 'images', '*.*'))),
        (os.path.join('share', package_name, package_name, 'audios'), glob(os.path.join(package_name, 'audios', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ana',
    maintainer_email='anachristinaac@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'screen_applications_node = screen_applications.screen_applications_node:main',
            'blank_screen_node = screen_applications.blank_screen_node:main',
            'screen_experiment2_node = screen_applications.screen_experiment2_node:main',
            'screen2_experiment2_node = screen_applications.screen2_experiment2_node:main',
        ],
    },
)
