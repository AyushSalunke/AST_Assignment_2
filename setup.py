from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ayush',
    maintainer_email='ayush16salunke@gmail.com',
    description='safety feature implementation using behavior tree and state machine',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_safety_behaviour_tree = my_pkg.robot_safety_behaviour_tree:main',
            'robot_safety_state_machine = my_pkg.robot_safety_state_machine:main',
            'test_behaviours = my_pkg.test_behaviours:main'
        ],
    },
)
