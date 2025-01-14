from setuptools import setup
import os
from glob import glob

package_name = 'pyqt_login'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyQt5'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='kwilee0426@gmail.com',
    description='A ROS2 package with a PyQt5 login window and error email notifications',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'login_node = pyqt_login.login_node:main',
            'error_publisher = pyqt_login.error_publisher:main',
            'robot_node = pyqt_login.robot_node:main',
        ],
    },
)
