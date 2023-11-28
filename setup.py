from setuptools import setup
from glob import glob
import os

package_name = 'offboard_path'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmccar',
    maintainer_email='rmccar@unm.edu',
    description='Package for flying fully-actuated multirotor and Omnicopter at UNM MARHES lab.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_ctrl_launch = offboard_path.offboard_ctrl.launch',
            'offboard_ctrl_omni = offboard_path.offboard_ctrl_omni:main',
            'offboard_ctrl_omni_params = offboard_path.offboard_ctrl_omni_params:main',
            'offboard_ctrl_omni_space = offboard_path.offboard_ctrl_omni_space:main',
            'offboard_ctrl_omni_space_reverse = offboard_path.offboard_ctrl_omni_space_reverse:main',
            'offboard_ctrl_quad = offboard_path.offboard_ctrl_quad:main',
            'offboard_ctrl_hex = offboard_path.offboard_ctrl_hex:main',
            'offboard_ctrl_hex_LQR = offboard_path.offboard_ctrl_hex_LQR:main',
            'offboard_ctrl_attitude = offboard_path.offboard_ctrl_attitude:main',
            'offboard_ctrl_quad_pos = offboard_path.offboard_ctrl_quad_pos:main',
            'odom_logger = offboard_path.odom_logger:main'
        ],
    },
)
