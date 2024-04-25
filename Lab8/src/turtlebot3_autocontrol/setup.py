from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_autocontrol'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='staszak.raf@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "traverse = turtlebot3_autocontrol.traverse:main",
            "count_objects = turtlebot3_autocontrol.count_objects:main"
        ]
    },
)
