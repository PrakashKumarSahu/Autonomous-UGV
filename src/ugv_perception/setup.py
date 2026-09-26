import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'ugv_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prakash Kumar Sahu',
    maintainer_email='prakashtech065@gmail.com',
    description='AI Perception stack: Depth Anything monocular depth and YOLO11/YOLOv8 hazard instance segmentation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'depth_node = ugv_perception.depth_node:main',
            'yolo_seg_node = ugv_perception.yolo_seg_node:main',
            'hazard_mask_node = ugv_perception.hazard_mask_node:main',
            'depth_to_pointcloud_node = ugv_perception.depth_to_pointcloud_node:main',
        ],
    },
)
