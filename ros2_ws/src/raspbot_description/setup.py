from setuptools import setup

package_name = 'raspbot_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/description.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/raspbot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@localhost',
    description='RASPBOT-V2 robot description (URDF) and static TF publisher',
    license='MIT',
)
