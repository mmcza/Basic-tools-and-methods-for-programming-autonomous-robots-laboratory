from setuptools import setup
import os
from glob import glob

package_name = 'ros_object_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Ros2 Object Detection package',
    license='MIT license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect = ros_object_detection.detect:main',
            'image_to_pcl = ros_object_detection.image_to_pcl:main',
            'publish_video = ros_object_detection.publish_video:main',
            'show_on_pcl = ros_object_detection.show_on_pcl:main',
            'show_tfs = ros_object_detection.show_tfs:main',
            'show_trajectory = ros_object_detection.show_trajectory:main',
        ],
    },
)
