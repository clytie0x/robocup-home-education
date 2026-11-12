from glob import glob
from setuptools import setup

package_name = 'voice_rviz_sim'

setup(
    name=package_name,
    version='0.2.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@example.com',
    description='RViz robot simulator driven by the voice_assistant flow',
    license='MIT',
    entry_points={
        'console_scripts': [
            'rviz_robot_sim = voice_rviz_sim.rviz_robot_sim:main',
        ],
    },
)
