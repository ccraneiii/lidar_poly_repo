from setuptools import find_packages, setup

from glob import glob  # change for being able to read a file from a standard location

package_name = 'lidar_poly'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),  # copy rviz config files
        ('share/' + package_name + '/yaml',  ['yaml/230731_pose_list_Tyndall.yaml']),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='carl',
    maintainer_email='carl.crane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'determine_lpoly = lidar_poly.determine_lidar_poly:main',
            'pose_service = lidar_poly.provide_pose_list_service:main',
            'veh_info = lidar_poly.provide_vehicle_position:main',
            'tf_broadcast = lidar_poly.transform_broadcast:main',
        ],
    },
)
