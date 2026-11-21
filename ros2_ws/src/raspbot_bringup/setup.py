from setuptools import setup
from glob import glob
import os

package_name = 'raspbot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'lidar'), glob('lidar/*')),
        (os.path.join('share', package_name, 'camera'), glob('camera/*')),
        (os.path.join('share', package_name, 'slam'), glob('slam/*')),
        (os.path.join('share', package_name, 'nav2'), glob('nav2/*')),
        (os.path.join('share', package_name, 'system'), glob('system/*')),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RASPBOT-V2 team',
    maintainer_email='raspbot@example.com',
    description='RASPBOT-V2 bringup launch files and configs',
    license='MIT',
)
