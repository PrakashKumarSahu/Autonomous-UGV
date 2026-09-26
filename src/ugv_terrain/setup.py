import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ugv_terrain'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prakash Kumar Sahu',
    maintainer_email='prakashtech065@gmail.com',
    description='2.5D Multi-layer Terrain Traversability Mapping and Occupancy Grid Generator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'terrain_analysis_node = ugv_terrain.terrain_analysis_node:main',
        ],
    },
)
