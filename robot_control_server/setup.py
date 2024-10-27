from setuptools import find_packages, setup

package_name = 'robot_control_server'

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
    maintainer='fudhail',
    maintainer_email='fudhail@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controll_server = robot_control_server.controll_server:main',
            'gripper_control = robot_control_server.gripper_control:main',
            'add_server = robot_control_server.add_server:main'
        ],
    },
)
