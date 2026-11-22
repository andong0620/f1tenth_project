"""ROS나 colcon 빌드 없이 `python3 -m pytest` 만으로 core 함수를 테스트하기 위한 설정."""
import pathlib
import sys

_ROOT = pathlib.Path(__file__).parent
for _package in ('perception', 'control', 'mode_manager', 'bringup'):
    sys.path.insert(0, str(_ROOT / 'src' / _package))
sys.path.insert(0, str(_ROOT / 'tools'))  # motor_sim
