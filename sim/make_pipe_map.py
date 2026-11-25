"""f1sim_ms(f1tenth_gym)용 파이프 트랙 비슷한 빈 맵(사각 고리 통로)을 만든다. 박스는 장애물 생성기로 놓는다.
팀 저장소 TODO/week12_LIDAR.md 4-1절의 스크립트와 같다. offline_sim.py 의 트랙과 모양이 같다.

    python3 sim/make_pipe_map.py maps/pipe              # 폭 1.2 m
    python3 sim/make_pipe_map.py maps/pipe --width 1.0  # 실측 통로 폭으로
"""
import argparse
import pathlib

import numpy as np

ap = argparse.ArgumentParser()
ap.add_argument('out', nargs='?', default='maps/pipe', help='확장자 없이')
ap.add_argument('--width', type=float, default=1.2, help='통로 폭 (m). 실측값으로 바꾼다')
ap.add_argument('--res', type=float, default=0.05, help='m/px')
args = ap.parse_args()

OUTER = (14.0, 8.0)  # 바깥 벽 가로, 세로 (m)
out = pathlib.Path(args.out)
out.parent.mkdir(parents=True, exist_ok=True)
w, h = int(OUTER[0] / args.res), int(OUTER[1] / args.res)
img = np.zeros((h, w), np.uint8)  # 0 = 벽(검정)
m = int(args.width / args.res)
img[1:-1, 1:-1] = 255             # 바깥 벽 안쪽은 비움
img[m:-m, m:-m] = 0               # 안쪽 섬은 벽

try:
    import cv2
    cv2.imwrite(str(out) + '.png', img)
except ImportError:  # OpenCV 가 없으면 Pillow 로
    from PIL import Image
    Image.fromarray(img).save(str(out) + '.png')

out.with_suffix('.yaml').write_text(
    f"image: {out.name}.png\nresolution: {args.res}\norigin: [0.0, 0.0, 0.0]\n"
    "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
    f"corridor_width: {args.width}  # make_obstacle_maps.py 가 읽는다\n")
print('saved', str(out) + '.png', '시작 위치 x=2.0, y=%.2f, theta=0' % (args.width / 2))
