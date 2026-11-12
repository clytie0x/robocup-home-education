from setuptools import setup
from glob import glob

package_name = 'voice_assistant'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    description='Offline STT -> LLM -> TTS voice interface for ROS 2 Jazzy',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'stt_node = voice_assistant.stt_node:main',
            'llm_node = voice_assistant.llm_node:main',
            'tts_node = voice_assistant.tts_node:main',
            'command_bridge_node = voice_assistant.command_bridge_node:main',
            'turtlesim_bag_sim = voice_assistant.turtlesim_bag_sim:main',
            'dialog_node = voice_assistant.dialog_node:main',
            'rule_node = voice_assistant.rule_node:main',
            'flow_node = voice_assistant.flow_node:main',
            'arm_action_bridge = voice_assistant.arm_action_bridge:main',
            'fake_arm_server = voice_assistant.fake_arm_server:main',
        ],
    },
)
