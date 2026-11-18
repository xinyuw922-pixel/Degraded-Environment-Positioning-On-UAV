"""Review a completed recording without fitting or correcting its trajectory."""

import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path

import numpy as np
import rosbag


class RecordingReview:
    """Extract recorded odometry and sparse stop statistics into a new analysis."""

    def __init__(self, run_directory, output_root):
        self.run_directory = run_directory.resolve()
        identifier = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%dT%H%M%S%z')
        self.directory = output_root.resolve() / (identifier + '_' + self.run_directory.name + '_review')

    def run(self):
        """Validate input hashes and save raw-frame statistics; no absolute ATE."""
        manifest = json.loads((self.run_directory / 'manifest.json').read_text())
        if manifest['status'] not in ('completed', 'aborted'):
            raise ValueError('Recording must be closed before review')
        self.directory.mkdir(parents=True, exist_ok=False)
        checksums = json.loads((self.run_directory / 'checksums.json').read_text())
        verification = {}
        for relative, expected in checksums.items():
            path = self.run_directory / relative
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for block in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(block)
            verification[relative] = digest.hexdigest() == expected['sha256']
        (self.directory / 'input_integrity.json').write_text(json.dumps(verification, indent=2)+'\n')
        if not all(verification.values()):
            raise ValueError('Input checksum mismatch; inspect input_integrity.json')
        arrivals, stamps, positions, rotations, frames = [], [], [], [], set()
        with rosbag.Bag(str(self.run_directory / 'raw/sensors.bag')) as recording:
            for topic, message, received in recording.read_messages(topics=['/fastlio/Odometry']):
                position = message.pose.pose.position
                rotation = message.pose.pose.orientation
                arrivals.append(received.to_sec())
                stamps.append(message.header.stamp.to_sec())
                positions.append((position.x, position.y, position.z))
                rotations.append((rotation.w, rotation.x, rotation.y, rotation.z))
                frames.add((message.header.frame_id, message.child_frame_id))
        if len(frames) != 1 or not positions:
            raise ValueError('Odometry must have a single frame pair and nonempty data')
        arrivals, stamps = np.asarray(arrivals), np.asarray(stamps)
        positions, rotations = np.asarray(positions), np.asarray(rotations)
        if not np.isfinite(positions).all() or not np.isfinite(rotations).all():
            raise ValueError('Nonfinite odometry; cannot silently discard it')
        np.savez_compressed(self.directory / 'trajectory.npz', arrival_unix_s=arrivals, source_stamp_s=stamps, positions_m=positions, rotations_wxyz=rotations)
        stops = []
        for stop in csv.DictReader((self.run_directory / 'reference/stop_observations.csv').open()):
            end, start = float(stop['end_unix_s']), float(stop['start_unix_s'])
            selected = positions[(arrivals >= end-5) & (arrivals <= end)]
            eligible = end-start >= 10 and 'incomplete' not in stop['notes'] and len(selected) > 1
            entry = dict(point_id=stop['point_id'], start_unix_s=start, end_unix_s=end, duration_s=end-start, selected_samples=len(selected), window_s=5, eligible_for_stop_summary=eligible, settled_independently_confirmed=stop['settled'] or None)
            entry.update(round_index=int(stop['round_index']) if stop.get('round_index') else None, visit_index=int(stop['visit_index']) if stop.get('visit_index') else None)
            if len(selected):
                mean = selected.mean(axis=0)
                entry.update(mean_position_m=mean.tolist(), xyz_std_m=selected.std(axis=0).tolist(), radial_p95_m=float(np.percentile(np.linalg.norm(selected-mean, axis=1), 95)), xyz_range_m=np.ptp(selected, axis=0).tolist())
            stops.append(entry)
        edges = []
        for first, second in zip(stops, stops[1:]):
            if first['eligible_for_stop_summary'] and second['eligible_for_stop_summary']:
                delta = np.subtract(second['mean_position_m'], first['mean_position_m'])
                edges.append(dict(from_point=first['point_id'], to_point=second['point_id'], distance_3d_m=float(np.linalg.norm(delta)), distance_xy_m=float(np.linalg.norm(delta[:2])), delta_z_m=float(delta[2])))
        intervals = np.diff(stamps)
        closure = None
        if len(stops) > 1 and stops[0]['point_id'] == stops[-1]['point_id'] and all(stop['eligible_for_stop_summary'] for stop in (stops[0], stops[-1])):
            closure = float(np.linalg.norm(np.subtract(stops[-1]['mean_position_m'], stops[0]['mean_position_m'])))
        summary = dict(schema_version='positioning-review-v0.1', source_run=str(self.run_directory), input_hashes_verified=True, odometry_messages=len(positions), frame_pair=list(next(iter(frames))), duration_s=float(arrivals[-1]-arrivals[0]), source_stamp_nonincreasing=int(np.sum(intervals <= 0)), source_stamp_interval_median_s=float(np.median(intervals)), source_stamp_interval_max_s=float(np.max(intervals)), position_range_m=np.ptp(positions, axis=0).tolist(), stops=stops, consecutive_stop_distances=edges, original_reference_id=manifest.get('reference_id'), absolute_ate_m=None, relative_pose_error=None, closure_error_m=closure, limitations=['No independent continuous truth or frozen room alignment', 'Distances are between raw odometry origins, not corrected rig_base', 'Stop windows use host bag receive time', 'Closure requires repeated endpoint labels and verified physical placement', 'Physical reference uncertainty and orientation consistency not documented'])
        (self.directory / 'metrics.json').write_text(json.dumps(summary, indent=2)+'\n')
        print(json.dumps(dict(analysis_directory=str(self.directory), **summary), indent=2))
        return self.directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_directory', type=Path)
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'positioning_data' / 'analyses')
    arguments = parser.parse_args()
    RecordingReview(arguments.run_directory, arguments.output_root).run()


if __name__ == '__main__':
    main()
