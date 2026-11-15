from setuptools import setup

setup(
    name='so100_driver',
    version='1.0.0',
    packages=['so100_driver'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/so100_driver']),
                ('share/so100_driver', ['package.xml'])],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': [
        'driver = so100_driver.driver_node:main',
        'calibrate = so100_driver.calibrate_cli:main',
        'record_motion = so100_driver.record_motion:main',
        'motion_test = so100_driver.motion_test:main',
        'scene_setup = so100_driver.scene_setup:main',
        'control_panel = so100_driver.control_panel:main',
        'pick_place = so100_driver.pick_place_node:main',
        'arm_cli = so100_driver.arm_client:main',
        'api_selftest = so100_driver.api_selftest:main',
        'camera_test = so100_driver.camera_test:main',
    ]},
)
