from setuptools import find_packages, setup
from glob import glob

package_name = 'multimedia_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='filip scopulovic',
    maintainer_email='f.scopulovic@gmail.com',
    description='This packages contains all Python-based nodes for the multimedia station acting as a radio station and Spotify client.',
    license='GPL-3.0-only',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'controll_node = multimedia_python.controll_node:main',
            'lcd_1602_node = multimedia_python.lcd_1602_node:main',
            'si470x_node = multimedia_python.si470x_node:main',
            'ir_receiver_node = multimedia_python.ir_receiver_node:main',
            'volume_system_node = multimedia_python.volume_system_node:main',
        ],
    },
)
