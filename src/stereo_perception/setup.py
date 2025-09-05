from setuptools import setup, find_packages

package_name = 'stereo_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'stereo_perception/launch/demo_stereo.launch.py'
        ]),
        ('share/' + package_name + '/calib', [
            'stereo_perception/calib/stereo_intrinsics.yaml',
            'stereo_perception/calib/stereo_extrinsics.yaml'
        ]),
        ('share/' + package_name + '/data', [
            'stereo_perception/data/left.png',
            'stereo_perception/data/right.png'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Stereo calibration, depth, and point cloud (Python + ROS2).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'file_image_publisher = stereo_perception.nodes.file_image_publisher:main',
            'stereo_rectify_node = stereo_perception.nodes.stereo_rectify_node:main',
            'depth_sgbm_node   = stereo_perception.nodes.depth_sgbm_node:main',
            'pointcloud_node   = stereo_perception.nodes.pointcloud_node:main',
        ],
    },
)
