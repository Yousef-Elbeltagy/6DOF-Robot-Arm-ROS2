from setuptools import find_packages, setup

package_name = 'robot_arm_python'

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
    maintainer='yousef_elbeltagy',
    maintainer_email='yousef_elbeltagy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_to_xyz = robot_arm_python.move_to_xyz:main',
             'robot_gui = robot_arm_python.robot_gui:main',

        ],
    },
)
