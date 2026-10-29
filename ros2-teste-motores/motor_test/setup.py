from glob import glob

from setuptools import find_packages, setup

package_name = 'motor_test'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Arthur Fontinele',
    maintainer_email='arthurfontis17@gmail.com',
    description='Sistema de teste dos motores do AUV (All Blue)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_test.motor_control_node:main',
            'gui_node = motor_test.gui_node:main',
        ],
    },
)
