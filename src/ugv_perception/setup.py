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
            # ── Depth Sources (select ONE via camera_type:= launch arg) ──────
            # sim:       camera_type:=sim        (Gazebo real depth camera)
            # hw mono:   camera_type:=monocular  (Depth Anything V3)
            # hw stereo: camera_type:=realsense  (Intel RealSense D435/D455)
            # hw stereo: camera_type:=zed        (ZED 2 / ZED X)
            'depth_relay_node = ugv_perception.depth_relay_node:main',
            'realsense_relay_node = ugv_perception.realsense_relay_node:main',
            'zed_relay_node = ugv_perception.zed_relay_node:main',
            # ── Monocular Depth Estimation (Depth Anything V3) ────────────────
            'depth_node = ugv_perception.depth_node:main',
            # ── Perception AI ─────────────────────────────────────────────────
            'yolo_seg_node = ugv_perception.yolo_seg_node:main',
            'hazard_mask_node = ugv_perception.hazard_mask_node:main',
            'depth_to_pointcloud_node = ugv_perception.depth_to_pointcloud_node:main',
        ],
    },
)
