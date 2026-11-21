from setuptools import setup

package_name = 'raspbot_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/fusion.launch.py']),
        ('share/' + package_name + '/config', ['config/ekf.yaml', 'config/limiter.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@localhost',
    description='RASPBOT-V2 fusion stack: velocity limiter, EKF config, odom relay',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'cmd_vel_limiter = raspbot_fusion.limiter_node:main',
            'odom_relay = raspbot_fusion.odom_relay:main',
        ],
    },
)
