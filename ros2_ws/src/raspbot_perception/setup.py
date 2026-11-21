import os
from glob import glob

from setuptools import setup

package_name = 'raspbot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
        (os.path.join('share', package_name, 'tools'),
         glob('tools/*.py')),
        (os.path.join('share', package_name, 'tools'),
         glob('tools/*.txt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RASPBOT-V2 team',
    maintainer_email='raspbot@example.com',
    description='RASPBOT-V2 WP-I: Hailo-8 YOLOv8 perception',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'hailo_node = raspbot_perception.hailo_node:main',
        ],
    },
)
