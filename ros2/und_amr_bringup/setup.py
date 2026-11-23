from glob import glob
from setuptools import setup

package_name = 'und_amr_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.xml')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='und_amr1',
    maintainer_email='undforbidding@gmail.com',
    description='UND_AMR bringup',
    license='MIT',
    entry_points={
        'console_scripts': [
            'base_node = und_amr_bringup.base_node:main',
            'api_node = und_amr_bringup.api_node:main',
            'imu_adapter = und_amr_bringup.imu_adapter:main',
            'cart_detector = und_amr_bringup.cart_detector:main',
            'scan_merger = und_amr_bringup.scan_merger:main',
            'camera_obstacles = und_amr_bringup.camera_obstacles:main',
            'map_odom_smoother = und_amr_bringup.map_odom_smoother:main',
        ],
    },
)
