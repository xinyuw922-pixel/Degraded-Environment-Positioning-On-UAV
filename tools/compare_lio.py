"""Compare replayed LIO candidates using the same FAST-LIO2 stop windows."""

import argparse
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path

import numpy as np

from utils.plane_utils import quaternion_matrix


class LioComparison:
    """Use identical source-time windows, preserving tentative grid correspondence."""

    def __init__(self, reference, replay, candidate_label='Point-LIO'):
        self.reference = reference
        self.replay = replay
        self.candidate_label = candidate_label

    @staticmethod
    def create_replayed_reference(original_analysis, replay, output):
        """Replace online reference poses with a cold replay, retaining time windows."""
        result = json.loads((replay/'result.json').read_text())
        if result['status'] != 'completed':
            raise ValueError('Reference replay did not complete')
        original = np.load(original_analysis/'trajectory.npz')
        metrics = json.loads((original_analysis/'metrics.json').read_text())
        rows = np.load(replay/'odometry.npy')
        if len(rows) < 2 or np.any(np.diff(rows[:, 0]) <= 0) or not np.isfinite(rows).all():
            raise ValueError('Invalid replayed reference')
        if original['source_stamp_s'][-1]-rows[-1, 0] > .5:
            raise ValueError('Reference replay did not cover the recording end')
        for stop in metrics['stops']:
            bounds = np.interp([stop['end_unix_s']-5, stop['end_unix_s']], original['arrival_unix_s'], original['source_stamp_s'])
            selected = rows[(rows[:, 0]>=bounds[0]) & (rows[:, 0]<=bounds[1]), 1:4]
            if len(selected) < 10:
                raise ValueError('Missing reference stop window')
            mean = selected.mean(axis=0)
            stop.update(mean_position_m=mean.tolist(), xyz_std_m=selected.std(axis=0).tolist(), radial_p95_m=float(np.percentile(np.linalg.norm(selected-mean, axis=1),95)), selected_samples=len(selected))
        metrics.update(cold_start=True, replay_source=str(replay), odometry_messages=len(rows), algorithm_label='FAST-LIO2 cold replay')
        output.mkdir(parents=True, exist_ok=False)
        (output/'metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
        np.savez_compressed(output/'trajectory.npz',positions_m=rows[:,1:4],rotations_wxyz=rows[:,4:8],source_stamp_s=rows[:,0],arrival_unix_s=np.interp(rows[:,0],original['source_stamp_s'],original['arrival_unix_s']))
        return output

    @staticmethod
    def _multiply(first, second):
        w1, vector1 = first[0], first[1:]
        w2, vector2 = second[0], second[1:]
        return np.r_[w1*w2-np.dot(vector1, vector2), w1*vector2+w2*vector1+np.cross(vector1, vector2)]

    @staticmethod
    def _repeat_summary(stops):
        if len(stops) != 24:
            raise ValueError('This comparison requires the recorded 12-point two-round run')
        first = np.array([stop['mean_position_m'] for stop in stops[:12]])
        second = np.array([stop['mean_position_m'] for stop in stops[12:]])
        # Same operator-order hypothesis for both algorithms, no optimized matching.
        correspondence = np.r_[np.arange(8), [11, 10, 9, 8]]
        delta = second-first[correspondence]
        distances = np.linalg.norm(delta, axis=1)
        return dict(repeat_rmse_m=float(np.sqrt(np.mean(distances**2))), repeat_max_m=float(max(distances)), first_8_repeat_rmse_m=float(np.sqrt(np.mean(distances[:8]**2))), per_point_repeat_distance_m=distances.tolist(), raw_z_range_m=float(np.ptp(np.r_[first[:, 2], second[:, 2]])), repeat_z_difference_rmse_m=float(np.sqrt(np.mean(delta[:, 2]**2))), stop_radial_p95_range_m=[min(stop['radial_p95_m'] for stop in stops), max(stop['radial_p95_m'] for stop in stops)])

    def run(self):
        result = json.loads((self.replay/'result.json').read_text())
        if result['status'] != 'completed':
            raise ValueError('Replay did not complete')
        reference = json.loads((self.reference/'metrics.json').read_text())
        original = np.load(self.reference/'trajectory.npz')
        rows = np.load(self.replay/'odometry.npy')
        preparation = dict(stop_sampling='interpolated at identical FAST-LIO2 reference header timestamps')
        if self.candidate_label == 'DLIO':
            original_count = len(rows)
            rows = rows[rows[:, 0] >= original['source_stamp_s'][0]+5]
            if np.any(np.diff(rows[:, 0]) < 0):
                raise ValueError('DLIO output timestamps moved backwards')
            keep = np.r_[np.diff(rows[:, 0]) != 0, True]
            preparation = dict(raw_rows=original_count, initial_5s_excluded=True, reason='Upstream 3s IMU calibration and startup; no evaluated stop in prefix', repeated_stamp_rows_removed=int(np.sum(~keep)), raw_odometry_preserved=True, stop_sampling='interpolated at the same FAST-LIO2 reference header timestamps')
            rows = rows[keep]
        if len(rows) < 2 or not np.isfinite(rows).all() or np.any(np.diff(rows[:, 0]) <= 0):
            raise ValueError('Replay odometry must be finite with increasing timestamps')
        if original['source_stamp_s'][-1]-rows[-1, 0] > .5:
            raise ValueError('Replay output ended before the input trajectory')
        # A single initial-pose transform is for display, never a best-fit trajectory.
        reference_index = int(np.argmin(abs(original['source_stamp_s']-rows[0, 0])))
        conjugate = rows[0, 4:8].copy(); conjugate[1:] *= -1
        rotation_quaternion = self._multiply(original['rotations_wxyz'][reference_index], conjugate)
        rotation = quaternion_matrix(rotation_quaternion)
        translation = original['positions_m'][reference_index]-rotation@rows[0, 1:4]
        displayed = rows[:, 1:4]@rotation.T+translation
        orientations = np.array([self._multiply(rotation_quaternion, quaternion) for quaternion in rows[:, 4:8]])
        stops, raw_stops = [], []
        for observation in reference['stops']:
            host_bounds = [observation['end_unix_s']-5, observation['end_unix_s']]
            bounds = np.interp(host_bounds, original['arrival_unix_s'], original['source_stamp_s'])
            mask = (rows[:, 0] >= bounds[0]) & (rows[:, 0] <= bounds[1])
            if mask.sum() < 10:
                raise ValueError('Insufficient replay coverage at '+observation['point_id'])
            selected = displayed[mask]
            reference_mask = (original['arrival_unix_s']>=host_bounds[0]) & (original['arrival_unix_s']<=host_bounds[1])
            timestamps = original['source_stamp_s'][reference_mask]
            selected = np.column_stack([np.interp(timestamps, rows[:,0], displayed[:,axis]) for axis in range(3)])
            entry = dict(observation)
            mean = selected.mean(axis=0)
            entry.update(mean_position_m=mean.tolist(), xyz_std_m=selected.std(axis=0).tolist(), radial_p95_m=float(np.percentile(np.linalg.norm(selected-mean, axis=1), 95)), selected_samples=len(selected), native_samples_in_window=int(mask.sum()), source_window_s=bounds.tolist())
            stops.append(entry)
            raw_entry = dict(entry)
            raw_entry['mean_position_m'] = np.column_stack([np.interp(timestamps,rows[:,0],rows[:,axis]) for axis in range(1,4)]).mean(axis=0).tolist()
            raw_stops.append(raw_entry)
        timestamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%dT%H%M%S%z')
        output = self.reference.parent/(timestamp+'_'+self.candidate_label.replace('-','')+'_FASTLIO2_comparison')
        output.mkdir(exist_ok=False)
        initialization = 'Both algorithms initialized from the same raw bag beginning' if reference.get('cold_start') else 'FAST-LIO2 was initialized online before recording; Point-LIO starts at first bag frame'
        comparison = dict(FAST_LIO2=self._repeat_summary(reference['stops']), Point_LIO_in_common_initial_frame=self._repeat_summary(stops), Point_LIO_raw_frame=self._repeat_summary(raw_stops), source_reference=str(self.reference), source_replay=str(self.replay), alignment=dict(method='single first overlapping pose; display comparison only', rotation_wxyz=rotation_quaternion.tolist(), translation_m=translation.tolist()), correspondence='Round 1 P09-P12 reversed hypothesis, applied identically to both', initialization_condition=initialization, limitations=['No independent continuous ground truth; not absolute accuracy', initialization, 'Recorded bag was replayed at accelerated speed; not a real-time performance test', 'IMU body origin used; no surveyed rig_base transform'])
        comparison['candidate_label'] = self.candidate_label
        comparison['preparation'] = preparation
        comparison['Candidate_in_common_initial_frame'] = comparison.pop('Point_LIO_in_common_initial_frame')
        comparison['Candidate_raw_frame'] = comparison.pop('Point_LIO_raw_frame')
        if self.candidate_label == 'Point-LIO':
            comparison['Point_LIO_in_common_initial_frame'] = comparison['Candidate_in_common_initial_frame']
            comparison['Point_LIO_raw_frame'] = comparison['Candidate_raw_frame']
        metrics = dict(reference)
        metrics.update(stops=stops, algorithm_label=self.candidate_label, frame_pair=['common_initial_FASTLIO2_frame', 'body'], comparison=comparison, consecutive_stop_distances=[], odometry_messages=len(rows), position_range_m=np.ptp(displayed, axis=0).tolist())
        (output/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
        (output/'comparison.json').write_text(json.dumps(comparison, indent=2)+'\n')
        np.savez_compressed(output/'trajectory.npz', positions_m=displayed, rotations_wxyz=orientations, source_stamp_s=rows[:, 0], arrival_unix_s=np.interp(rows[:, 0], original['source_stamp_s'], original['arrival_unix_s']))
        np.savez_compressed(output/'reference_trajectory.npz', positions_m=original['positions_m'])
        lines = ['**{} vs FAST-LIO2** (same raw recording)'.format(self.candidate_label), '', '| Metric | FAST-LIO2 | {} |'.format(self.candidate_label), '|---|---:|---:|']
        first, second = comparison['FAST_LIO2'], comparison['Candidate_in_common_initial_frame']
        for name, key in [('Two-round 3D repeat RMSE (cm)', 'repeat_rmse_m'), ('Two-round max difference (cm)', 'repeat_max_m'), ('First 8 points repeat RMSE (cm)', 'first_8_repeat_rmse_m'), ('Stop Z range, common frame (cm)', 'raw_z_range_m')]:
            lines.append('| {} | {:.2f} | {:.2f} |'.format(name, 100*first[key], 100*second[key]))
        lines += ['', 'Teal: {}. Purple: FAST-LIO2 reference. Initial-pose alignment only; no fitted trajectory.'.format(self.candidate_label), '', initialization+'. Round-1 third row uses the same reversed-order hypothesis. Exploratory repeatability results, not accuracy acceptance.']
        (output/'operator_annotation.json').write_text(json.dumps({'display_note':'\n'.join(lines)}, indent=2)+'\n')
        Path('/tmp/pointlio-comparison-path').write_text(str(output))
        print(json.dumps(dict(output=str(output), **comparison), indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    parser.add_argument('--replay', type=Path, required=True)
    parser.add_argument('--candidate-label', choices=['Point-LIO','DLIO','Super-LIO','RKO-LIO','KISS-ICP','GLIM','Traj-LO','CLID-SLAM'], default='Point-LIO')
    arguments = parser.parse_args()
    LioComparison(arguments.reference, arguments.replay, arguments.candidate_label).run()


if __name__ == '__main__':
    main()
