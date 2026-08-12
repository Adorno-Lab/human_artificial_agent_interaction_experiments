import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'interaction_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'log'),
         glob(os.path.join(package_name, 'log', '*.log*'))),
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
            'interaction_manager_node = interaction_manager.interaction_manager_node:main',
            'manager_experiment2_node = interaction_manager.manager_experiment2_node:main'
        ],
    },
)
