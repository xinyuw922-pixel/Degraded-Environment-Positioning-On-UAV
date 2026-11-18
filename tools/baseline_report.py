"""Collect completed cold-replay comparisons without inventing pending results."""

import argparse
import json
from pathlib import Path
import shutil

import numpy as np


class BaselineReport:
    """Create a versioned nine-method table and optional viser trajectory bundle."""

    METHODS = ['FAST-LIO2', 'Point-LIO', 'DLIO', 'RKO-LIO', 'Super-LIO',
               'GLIM', 'KISS-ICP', 'Traj-LO', 'CLID-SLAM']
    COLORS = {'Point-LIO': [255, 180, 60], 'RKO-LIO': [240, 80, 140],
              'Super-LIO': [80, 210, 90], 'GLIM': [80, 130, 250],
              'KISS-ICP': [230, 210, 50], 'Traj-LO': [240, 110, 50],
              'CLID-SLAM': [200, 200, 200]}

    def __init__(self, arguments):
        self.arguments = arguments

    def run(self):
        root = self.arguments.comparison_root.resolve()
        output = self.arguments.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        latest = {}
        for path in sorted(root.glob('*/comparison.json')):
            comparison = json.loads(path.read_text())
            if 'candidate_label' in comparison and 'Candidate_in_common_initial_frame' in comparison:
                latest[comparison['candidate_label']] = (path.parent, comparison)
        if not latest:
            raise ValueError('No completed comparisons found')
        baseline = next(iter(latest.values()))[1]['FAST_LIO2']
        rows, trajectories, colors = [], {}, {}
        primary = latest['DLIO'][0] if 'DLIO' in latest else next(iter(latest.values()))[0]
        for name in self.METHODS:
            if name == 'FAST-LIO2':
                metrics = baseline
                directory = None
            elif name in latest:
                directory, comparison = latest[name]
                metrics = comparison['Candidate_in_common_initial_frame']
                if directory != primary:
                    trajectories[name] = np.load(directory/'trajectory.npz')['positions_m']
                    colors[name] = self.COLORS.get(name, [255, 180, 60])
            else:
                rows.append(dict(method=name, status='pending_full_comparison'))
                continue
            rows.append(dict(method=name, status='completed_full_comparison',
                             comparison_directory=str(directory) if directory else None,
                             **metrics))
        limits = ['One 4x3 two-round recording; no independent ground truth',
                  'Round-1 third-row reversal remains an operator-order hypothesis',
                  'Identical final-5s windows and FAST reference sample timestamps',
                  'Common IMU origin; single initial-pose alignment only',
                  'Upstream default estimator parameters differ; no parameter search',
                  'Concurrent offline jobs do not constitute a real-time benchmark']
        (output/'summary.json').write_text(json.dumps(dict(methods=rows, limitations=limits), indent=2)+'\n')
        lines = ['# Nine-method cold replay comparison', '',
                 'Values below measure repeated placement, not absolute accuracy.', '',
                 '| Method | Status | Repeat 3D RMSE (cm) | Max (cm) | Common-frame Z range (cm) |',
                 '|---|---|---:|---:|---:|']
        for row in rows:
            values = ['{:.2f}'.format(100*row[key]) for key in
                      ('repeat_rmse_m', 'repeat_max_m', 'raw_z_range_m')] if 'repeat_rmse_m' in row else ['—']*3
            lines.append('| {} | {} | {} |'.format(row['method'], row['status'], ' | '.join(values)))
        lines += ['', *['- '+limit for limit in limits]]
        (output/'report.md').write_text('\n'.join(lines)+'\n')
        for filename in ('metrics.json', 'trajectory.npz', 'reference_trajectory.npz'):
            shutil.copy2(primary/filename, output/filename)
        np.savez_compressed(output/'additional_trajectories.npz', **trajectories)
        (output/'additional_trajectories.json').write_text(json.dumps(colors, indent=2)+'\n')
        (output/'operator_annotation.json').write_text(json.dumps(
            {'display_note': '\n'.join(lines[2:])}, indent=2)+'\n')
        print(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    BaselineReport(parser.parse_args()).run()


if __name__ == '__main__':
    main()
