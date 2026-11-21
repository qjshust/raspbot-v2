from setuptools import setup

package_name = 'raspbot_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu.launch.py']),
        ('share/' + package_name + '/config', ['config/imu.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@localhost',
    description='IMU driver and attitude estimation for RASPBOT-V2 (ICM-42688-P / HWT101)',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'imu_node = raspbot_imu.node:main',
            'imu_demo = raspbot_imu.demo:main',
        ],
    },
)
