from setuptools import setup
import os
from glob import glob

package_name = 'user_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
       
        ('share/' + package_name, ['package.xml']),
       
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
       
        (os.path.join('share', package_name, 'images'), glob('images/*')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'PyQt5',
        'opencv-python',
        'numpy',
        'python-dotenv',
    ],
    zip_safe=True,
    maintainer='hyuna',
    maintainer_email='sjajmh6612@naver.com',
    description='Combined GUI Application',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'user_gui = user_gui.user_gui:main',
            'aruco = user_gui.arUco_detect:main',
            'control = user_gui.control:main',
            'sort = user_gui.sort:main',
            'yolo_node = user_gui.yolo_node:main',  
            'teleop_test = user_gui.teleop:main',  
        ],
    },
)

