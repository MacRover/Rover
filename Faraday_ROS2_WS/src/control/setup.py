from setuptools import setup

package_name = 'control'

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
    maintainer='cho',
    maintainer_email='johnnychox@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'keyboard_drive = control.ros2_keyboard_drive_control:main',
		'cmd_repeater = control.cmd_repeater:main',
		'controller_drive = control.ros2_controller_drive:main'
        ],
    },
)
