"""Visualize the LiDAR planning stage on synthetic scans (no ROS needed).

    python demo.py   # writes docs/gap_follow_demo.png and docs/clustering_demo.png

The car is at the origin facing +x (REP-103: x forward, y left).
"""
import pathlib
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = pathlib.Path(__file__).parent
sys.path[:0] = [str(ROOT / 'src' / 'control'), str(ROOT / 'src' / 'perception')]

from control.core.gap import (  # noqa: E402
    best_point, disparity_extend, find_max_gap, preprocess, safety_bubble)
from perception.core.scan_utils import cluster_scan, crop_fov, find_obstacles  # noqa: E402
from perception.core.synth import beam_angles, box, corridor, raycast  # noqa: E402

# Same values as the team's test parameters.
GAP = dict(fov_deg=180.0, max_range=3.0, window=3, bubble_radius=0.3,
           disparity_threshold=0.2, half_width=0.2, depth_ratio=0.9, max_steer_deg=45.0)
CLUSTER = dict(break_dist=0.1, break_ratio=0.05)
OBSTACLE = dict(corridor_width=0.6, d_obs=2.0, max_obstacle_len=0.8)

DEAD_END = np.array([
    [-5.0, 1.0, 2.5, 1.0], [2.5, 1.0, 2.5, -6.0],
    [-5.0, -1.0, 1.0, -1.0], [1.0, -1.0, 1.0, -6.0]])

GAP_SCENES = {
    'Pillar ahead': np.vstack([corridor(3.0), box(1.5, 0.5, 0.2, 0.2)]),
    'Gap too narrow for the car': np.vstack([corridor(2.0), box(1.5, 0.6, 0.3, 0.3)]),
    'Dead end, exit on the right': DEAD_END,
}

CLUSTER_SCENES = {
    'Box in the corridor -> obstacle': np.vstack([corridor(2.0), box(1.5, 0.0, 0.3, 0.3)]),
    'Corner outer wall -> NOT an obstacle': np.array([
        [-5.0, -1.0, 1.5, -1.0], [1.5, -1.0, 1.5, 6.0], [-5.0, 1.0, -1.0, 1.0]]),
}


def draw_walls(ax, segments):
    for x1, y1, x2, y2 in segments:
        ax.plot([x1, x2], [y1, y2], color='0.2', lw=2.5, zorder=1)
    ax.plot(0, 0, marker='s', ms=10, color='black', zorder=5)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)


def gap_figure(path):
    fig, axes = plt.subplots(1, len(GAP_SCENES), figsize=(15, 4.8))
    for ax, (title, segments) in zip(axes, GAP_SCENES.items()):
        angles_all = beam_angles()
        ranges, angles = crop_fov(raycast(angles_all, segments), angles_all, GAP['fov_deg'])
        clean = preprocess(ranges, angles, GAP['max_range'], GAP['window'])
        bubbled = safety_bubble(clean, angles, GAP['bubble_radius'])
        free = disparity_extend(bubbled, angles, GAP['disparity_threshold'], GAP['half_width'])
        start, end = find_max_gap(free)
        target = best_point(free, start, end, GAP['depth_ratio'])
        steer = np.clip(angles[target], -np.radians(GAP['max_steer_deg']),
                        np.radians(GAP['max_steer_deg']))

        def xy(r, mask=None):
            r = r if mask is None else np.where(mask, r, np.nan)
            return r * np.cos(angles), r * np.sin(angles)

        draw_walls(ax, segments)
        ax.scatter(*xy(clean), s=2, color='0.65', label='Scan after preprocess (clipped at 3 m)', zorder=2)
        ax.scatter(*xy(clean, (bubbled == 0) & (clean > 0)), s=4, color='tab:orange',
                   label='Safety bubble', zorder=3)
        ax.scatter(*xy(clean, (free == 0) & (bubbled > 0)), s=4, color='tab:red',
                   label='Disparity extender', zorder=3)
        in_gap = np.zeros_like(free, dtype=bool)
        in_gap[start:end + 1] = True
        ax.scatter(*xy(free, in_gap), s=4, color='tab:green', label='Largest free gap', zorder=3)
        ax.annotate('', xy=(np.cos(steer), np.sin(steer)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle='-|>', color='tab:blue', lw=2.5), zorder=6)
        ax.set_title(f'{title}\nsteering {np.degrees(steer):+.1f}°', fontsize=11)
        ax.set_xlim(-0.6, 3.2)
        ax.set_ylim(-1.9, 1.9)
    axes[0].legend(loc='lower left', fontsize=8, markerscale=3, framealpha=0.95)
    fig.suptitle('Follow-the-Gap with disparity extender (blue arrow = steering target)', fontsize=13)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def cluster_figure(path):
    fig, axes = plt.subplots(1, len(CLUSTER_SCENES), figsize=(11, 5))
    colors = plt.cm.tab10.colors
    for ax, (title, segments) in zip(axes, CLUSTER_SCENES.items()):
        angles = beam_angles()
        ranges = raycast(angles, segments)
        clusters = cluster_scan(ranges, angles, **CLUSTER)
        obstacles = find_obstacles(clusters, **OBSTACLE)

        draw_walls(ax, segments)
        for i, c in enumerate(clusters):
            is_obs = any(c is o for o in obstacles)
            ax.scatter(c.points[:, 0], c.points[:, 1], s=14 if is_obs else 3,
                       color='tab:red' if is_obs else colors[i % 10], zorder=3)
        w, d = OBSTACLE['corridor_width'] / 2.0, OBSTACLE['d_obs']
        ax.add_patch(plt.Rectangle((0, -w), d, 2 * w, fill=False, ls='--', color='tab:red', lw=1.5))
        ax.set_title(f'{title}\n{len(clusters)} clusters, {len(obstacles)} obstacle(s)', fontsize=11)
        ax.set_xlim(-0.6, 3.0)
        ax.set_ylim(-1.8, 2.2)
    fig.suptitle('Wall / obstacle classification per scan: colors = clusters, '
                 'red = obstacle, dashed box = search area', fontsize=12)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == '__main__':
    out = ROOT / 'docs'
    out.mkdir(exist_ok=True)
    gap_figure(out / 'gap_follow_demo.png')
    cluster_figure(out / 'clustering_demo.png')
    print('saved figures to', out)
