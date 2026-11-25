"""파이프 맵에 f1sim_ms 장애물 생성기(tools/obstacle_map_maker.py)로 박스를 구워 3종 맵을 만든다.

GUI 대신 생성기의 MapModel·Obstacle 클래스를 그대로 불러 써서, 매번 같은 배치를 재현한다.
결과는 GUI의 '맵만 저장'과 같다: <이름>.png, <이름>.yaml, <이름>.obstacles.yaml
(사이드카 파일이 있으니 GUI로 열어 박스를 옮겨 가며 더 만들 수도 있다)

    python3 sim/make_pipe_map.py maps/pipe --width 1.2
    python3 sim/make_obstacle_maps.py maps/pipe.yaml --maker ~/f1sim_ms/tools/obstacle_map_maker.py

만드는 맵 (팀 TODO의 3종. 통로 폭 W, 출발 직선 y 0~W 에 놓는다)
- pipe_obs_alt  : 좌우 번갈아 3개 (x 4, 7, 10)
- pipe_obs_row  : 같은 쪽에 연달아 2개, 중심 간격 1.0 m (x 5, 6)
- pipe_obs_wall : 바깥 벽에 붙인 1개 (x 6)
- pipe_obs_center : 통로 가운데를 막는 2개 (x 5, 9.5). 중앙을 따라가는 wall follow 로는 못 지나가서
                    AUTO 모드가 GAP 으로 바뀌는 것을 볼 때 쓴다
"""
import argparse
import importlib.util
import os
import pathlib

import yaml

ap = argparse.ArgumentParser()
ap.add_argument('map_yaml', help='make_pipe_map.py 로 만든 맵 yaml')
ap.add_argument('--maker', default=os.path.expanduser('~/f1sim_ms/tools/obstacle_map_maker.py'),
                help='f1sim_ms 의 장애물 생성기 경로')
ap.add_argument('--box', type=float, default=0.4, help='박스 한 변 (m). 실측값으로 바꾼다')
args = ap.parse_args()

spec = importlib.util.spec_from_file_location('obstacle_map_maker', args.maker)
omm = importlib.util.module_from_spec(spec)
spec.loader.exec_module(omm)  # 생성기는 tkinter 없이도 import 된다 (headless import 허용)

meta = yaml.safe_load(open(args.map_yaml, encoding='utf-8'))
width = float(meta.get('corridor_width', 1.2))
b, r = args.box, args.box / 2
low, high = r + 0.05, width - r - 0.05  # 바깥 벽 쪽, 안쪽 섬 쪽 박스 중심 y (벽과 5 cm)
layouts = {
    'alt': [(4.0, low), (7.0, high), (10.0, low)],
    'row': [(5.0, low), (6.0, low)],
    'wall': [(6.0, r)],
    'center': [(5.0, width / 2), (9.5, width / 2 + 0.15)],
}
stem = str(pathlib.Path(args.map_yaml).with_suffix(''))
for name, boxes in layouts.items():
    obstacles = [omm.Obstacle('rect', x, y, width=b, height=b) for x, y in boxes]
    out = omm.MapModel(args.map_yaml).save_baked(
        f'{stem}_obs_{name}', obstacles, start_pose=(2.0, width / 2, 0.0))
    print('saved', out[0], [(x, round(y, 2)) for x, y in boxes])
