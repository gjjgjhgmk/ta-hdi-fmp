#!/usr/bin/env python3
"""
TA-HDI-FMP 核心算法可视化测试
验证规划路径、障碍物、危险区标记
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.collections import LineCollection

import sys
sys.path.insert(0, '/home/muxi/ta_hdi_ws/install/ta_hdi_fmp_core')
sys.path.insert(0, '/home/muxi/ta_hdi_ws/src/ta_hdi_fmp_core/ta_hdi_fmp_core')

from ta_hdi_fmp_core import plan_path, PlannerConfig


def plot_scenario(ax, start, goal, obstacles, result, cfg):
    """绘制单个场景"""
    ax.clear()
    ax.set_aspect('equal')
    ax.set_xlim(-4, 4)
    ax.set_ylim(-3, 3)
    ax.grid(True, alpha=0.3)

    # 绘制障碍物
    for obs in obstacles:
        obs = list(obs) + [0] * (7 - len(obs))  # 补齐到7个元素
        cx, cy, radius, w, h, theta, obs_type = obs
        obs_type = int(obs_type) if obs_type else 1
        if obs_type == 1:  # 圆形
            circle = plt.Circle((cx, cy), radius, color='red', alpha=0.3)
            ax.add_patch(circle)
            circle_edge = plt.Circle((cx, cy), radius, fill=False,
                                      edgecolor='red', linewidth=2)
            ax.add_patch(circle_edge)
            ax.text(cx, cy, f'r={radius}', ha='center', va='center', fontsize=8)
        else:  # OBB 矩形
            rect = patches.Rectangle((cx - w/2, cy - h/2), w, h,
                                     angle=np.degrees(theta),
                                     rotation_point='center',
                                     linewidth=2, edgecolor='red',
                                     facecolor='red', alpha=0.3)
            ax.add_patch(rect)

    # 绘制危险区（safe_margin 范围）
    safe_margin = cfg.safe_margin
    for obs in obstacles:
        obs = list(obs) + [0] * (7 - len(obs))
        cx, cy, radius, w, h, theta, obs_type = obs
        if int(obs_type) == 1:
            danger = plt.Circle((cx, cy), radius + safe_margin,
                               color='yellow', alpha=0.15)
            ax.add_patch(danger)

    # 绘制路径
    if result['success']:
        path_xy = result['path_xy']
        if path_xy.size > 0:
            # 路径线
            ax.plot(path_xy[0, :], path_xy[1, :], 'g-', linewidth=2,
                   label='规划路径')

            # 路径点
            ax.plot(path_xy[0, :], path_xy[1, :], 'go', markersize=6)

            # 标注路径点
            for i in range(path_xy.shape[1]):
                ax.annotate(f'P{i}', (path_xy[0, i], path_xy[1, i]),
                           textcoords="offset points", xytext=(5, 5),
                           fontsize=8)

    # 绘制起点终点
    ax.plot(start[0], start[1], 'bs', markersize=12, label='起点')
    ax.plot(goal[0], goal[1], 'b^', markersize=12, label='终点')

    # 添加指标信息
    min_dist = result['metrics']['min_dist']
    kappa_max = result['metrics']['kappa_max']
    jerk = result['metrics']['jerk_rms']
    status = result['debug']['status']

    info_text = f"Status: {status}\n"
    info_text += f"min_dist: {min_dist:.3f}\n"
    info_text += f"kappa_max: {kappa_max:.3f}\n"
    info_text += f"jerk_rms: {jerk:.3f}"

    if result['success']:
        info_text += f"\n路径点数: {result['debug']['n_via']}"
        info_text += f"\n使用候选: {result['debug']['candidate_used']}"

    ax.text(0.02, 0.98, info_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    ax.legend(loc='upper right')
    ax.set_title(f"场景: 起点{start} → 终点{goal}")


def run_tests():
    """运行多个测试场景"""

    # 创建配置
    cfg = PlannerConfig()
    cfg.time_budget_per_segment = 5.0
    cfg.safe_margin = 0.5
    cfg.kappa_pass_thresh = 2.0
    cfg.jerk_pass_thresh = 500.0

    print("=" * 60)
    print("TA-HDI-FMP 核心算法测试")
    print("=" * 60)
    print(f"配置参数:")
    print(f"  - time_budget: {cfg.time_budget_per_segment}s")
    print(f"  - safe_margin: {cfg.safe_margin}")
    print(f"  - kappa_thresh: {cfg.kappa_pass_thresh}")
    print(f"  - jerk_thresh: {cfg.jerk_pass_thresh}")
    print("=" * 60)

    # 测试场景定义 (obstacles: [cx, cy, radius, w, h, theta, type])
    # type: 1=圆形, 2=矩形(OBB)
    scenarios = [
        {
            'name': '无障碍物',
            'start': [-2.0, -0.5],
            'goal': [2.0, -0.5],
            'obstacles': []
        },
        {
            'name': '单个圆形障碍物（中心）',
            'start': [-2.0, -0.5],
            'goal': [2.0, 0.0],
            'obstacles': [[0.0, 0.0, 0.8, 0, 0, 0, 1]]
        },
        {
            'name': '单个圆形障碍物（偏上）',
            'start': [-2.0, -0.5],
            'goal': [2.0, 2.0],
            'obstacles': [[0.0, 0.5, 0.8, 0, 0, 0, 1]]
        },
        {
            'name': '两个圆形障碍物',
            'start': [-2.0, -0.5],
            'goal': [2.0, 0.0],
            'obstacles': [
                [-1.0, 0.0, 0.6, 0, 0, 0, 1],
                [1.0, 0.0, 0.6, 0, 0, 0, 1]
            ]
        },
        {
            'name': '窄通道',
            'start': [-2.0, 0.0],
            'goal': [2.0, 0.0],
            'obstacles': [
                [-0.5, 1.0, 0.5, 0, 0, 0, 1],
                [0.5, -1.0, 0.5, 0, 0, 0, 1]
            ]
        },
        {
            'name': 'OBB 矩形障碍物',
            'start': [-2.0, -0.5],
            'goal': [2.0, 0.5],
            'obstacles': [[0.0, 0.0, 0, 1.5, 0.5, np.radians(45), 2]]
        },
    ]

    # 创建图形窗口
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    axes = axes.flatten()

    results = []

    for i, scenario in enumerate(scenarios):
        print(f"\n[场景 {i+1}] {scenario['name']}")
        print(f"  起点: {scenario['start']}, 终点: {scenario['goal']}")
        print(f"  障碍物: {scenario['obstacles']}")

        # 运行规划
        result = plan_path(scenario['start'], scenario['goal'],
                         scenario['obstacles'], cfg)

        results.append(result)

        print(f"  结果: success={result['success']}, "
              f"status={result['debug']['status']}")
        print(f"  指标: min_dist={result['metrics']['min_dist']:.3f}, "
              f"kappa={result['metrics']['kappa_max']:.3f}")

        if result['success']:
            print(f"  路径点形状: {result['path_xy'].shape}")
            print(f"  路径点数: {result['debug']['n_via']}")

        # 绘制
        plot_scenario(axes[i], scenario['start'], scenario['goal'],
                     scenario['obstacles'], result, cfg)
        axes[i].set_title(f"{i+1}. {scenario['name']}")

    plt.tight_layout()
    plt.savefig('/tmp/ta_hdi_fmp_test.png', dpi=150)
    print(f"\n图片已保存到: /tmp/ta_hdi_fmp_test.png")

    # 统计
    print("\n" + "=" * 60)
    print("测试统计")
    print("=" * 60)
    success_count = sum(1 for r in results if r['success'])
    print(f"成功率: {success_count}/{len(results)} = {success_count/len(results)*100:.1f}%")

    if success_count > 0:
        min_dists = [r['metrics']['min_dist'] for r in results if r['success']]
        kappas = [r['metrics']['kappa_max'] for r in results if r['success']]
        print(f"平均 min_dist: {np.mean(min_dists):.3f}")
        print(f"平均 kappa_max: {np.mean(kappas):.3f}")

    plt.show()

    return results


if __name__ == '__main__':
    results = run_tests()
