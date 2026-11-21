from setuptools import setup

package_name = 'raspbot_mock'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mock.launch.py']),
        ('share/' + package_name + '/config', ['config/mock.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@localhost',
    description='Mock nodes for RASPBOT-V2 offline integration (SW-I04)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fake_base = raspbot_mock.fake_base:main',
            'fake_imu = raspbot_mock.fake_imu:main',
            'fake_scan = raspbot_mock.fake_scan:main',
        ],
    },
)
