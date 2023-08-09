from setuptools import setup

package_name = 'offboard_path'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rmccar',
    maintainer_email='rmccar@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_ctrl_omni = offboard_path.offboard_ctrl_omni:main',
            'offboard_ctrl_quad = offboard_path.offboard_ctrl_quad:main',
            'offboard_ctrl_hex = offboard_path.offboard_ctrl_hex:main',
            'offboard_ctrl_attitude = offboard_path.offboard_ctrl_attitude:main',
            'odom_logger = offboard_path.odom_logger:main'
        ],
    },
)
