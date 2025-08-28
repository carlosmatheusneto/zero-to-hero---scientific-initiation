from setuptools import find_packages, setup

package_name = 'lidar_zed'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jooj',
    maintainer_email='joojmiguel54@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intrinsic_camera_calibration = lidar_zed.intrinsic_camera_calibration:main',
            'extrinsic_camera_calibration = lidar_zed.extrinsic_camera_calibration:main',
            'synchronizer = lidar_zed.synchronizer:main'
        ],
    },
)
