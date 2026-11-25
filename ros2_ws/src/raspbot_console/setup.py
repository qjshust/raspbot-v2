from glob import glob
from setuptools import setup

import os

package_name = 'raspbot_console'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RASPBOT-V2 team',
    maintainer_email='raspbot@example.com',
    description='RASPBOT-V2 Pi 端 HTTP/SSE 网关（Wire API v1）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'console_bridge = raspbot_console.bridge:main',
        ],
    },
)
