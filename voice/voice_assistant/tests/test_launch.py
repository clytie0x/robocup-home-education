"""The Help Me Carry launch file: it parses, it is installed, and it starts only what the task needs.

The launch itself needs ROS 2 (launch / launch_ros), which is not installed here, so it is checked as
source: python -m py_compile + ast, plus the node list in the LaunchDescription source.
"""
import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch' / 'hmc_voice.launch.py'


def test_hmc_voice_launch_parses():
    ast.parse(LAUNCH.read_text(encoding='utf-8'))            # what `python -c "ast.parse(...)"` does


def test_hmc_voice_launch_is_installed_like_the_existing_one():
    setup = (PACKAGE / 'setup.py').read_text(encoding='utf-8')
    assert "glob('launch/*.py')" in setup                    # every launch file in launch/ is installed


def test_hmc_voice_launch_starts_the_three_voice_nodes():
    src = LAUNCH.read_text(encoding='utf-8')
    executables = []
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.Call):
            for keyword in node.keywords:
                if keyword.arg == 'executable' and isinstance(keyword.value, ast.Constant):
                    executables.append(keyword.value.value)
    assert executables == ['stt_node', 'rule_node', 'tts_node']


def test_hmc_voice_launch_leaves_the_task_flow_alone():
    # hmc_flow.py owns /tts/say and /robot/command; flow_node and command_bridge_node would duplicate them
    src = LAUNCH.read_text(encoding='utf-8')
    for name in ('flow_node', 'command_bridge_node', 'llm_node'):
        assert f"executable='{name}'" not in src
