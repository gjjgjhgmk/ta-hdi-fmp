import csv
import time
from pathlib import Path


def main():
    out = Path('benchmark_results_ros2.csv')
    rows = [
        ['scenario_id', 'planning_time_ms', 'goal_reached', 'path_length', 'min_dist', 'kappa_p99', 'jerk_rms']
    ]
    # Placeholder runner; to be filled with service calls in VM runtime.
    for i in range(10):
        rows.append([i, 0.0, 0, 0.0, 0.0, 0.0, 0.0])
    with out.open('w', newline='', encoding='utf-8') as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
