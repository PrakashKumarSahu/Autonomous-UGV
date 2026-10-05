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
    description='AI perception stack: swappable depth sources (Gazebo/RealSense/ZED/Depth Anything V2) and YOLO26 hazard instance segmentation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # ── Depth sources (exactly ONE runs, selected by camera_type:=) ──
            #   sim        → depth_relay_node      (Gazebo depth camera)
            #   monocular  → depth_node            (Depth Anything V2 Metric)
            #   realsense  → realsense_relay_node  (Intel RealSense D435/D455)
            #   zed        → zed_relay_node        (ZED 2 / ZED X)
            'depth_relay_node = ugv_perception.depth_relay_node:main',
            'realsense_relay_node = ugv_perception.realsense_relay_node:main',
            'zed_relay_node = ugv_perception.zed_relay_node:main',
            'depth_node = ugv_perception.depth_node:main',
            # ── Always-on perception ─────────────────────────────────────────
            'depth_to_pointcloud_node = ugv_perception.depth_to_pointcloud_node:main',
            'yolo_seg_node = ugv_perception.yolo_seg_node:main',
        ],
    },
)
