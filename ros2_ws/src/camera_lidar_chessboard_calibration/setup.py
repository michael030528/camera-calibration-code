from setuptools import find_packages, setup
from glob import glob

package_name = 'camera_lidar_chessboard_calibration'
setup(
    name=package_name, version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'], zip_safe=True,
    maintainer='user', maintainer_email='user@localhost', license='MIT',
    entry_points={'console_scripts': [
        'camera_rtsp = camera_lidar_chessboard_calibration.camera_rtsp:main',
        'capture_gui = camera_lidar_chessboard_calibration.capture_gui:main',
        'livox_direct = camera_lidar_chessboard_calibration.livox_direct:main',
        'calibrate = camera_lidar_chessboard_calibration.calibrate:main',
    ]},
)
