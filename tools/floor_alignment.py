"""Try floor alignment from a held-out initial raw scan segment, not stop means."""

import argparse
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import shutil

import numpy as np
import rosbag

from utils.plane_utils import estimate_near_horizontal_plane, plane_level_transform, quaternion_matrix


class FloorAlignmentTrial:
    """Create a new diagnostic analysis and retain all original coordinates."""

    def __init__(self, source_analysis):
        self.source_analysis = source_analysis.resolve()
        self.metrics = json.loads((source_analysis / 'metrics.json').read_text())
        self.run_directory = Path(self.metrics['source_run'])
        self.trajectory = np.load(source_analysis / 'trajectory.npz')

    def run(self):
        """Estimate a plane from the first five scans and evaluate withheld stops."""
        clouds, associations = self._initial_clouds()
        candidates = np.concatenate(clouds)
        if len(candidates) > 15000:
            indices = np.random.default_rng(7).choice(len(candidates), 15000, replace=False)
            candidates = candidates[indices]
        normal, offset, inliers = estimate_near_horizontal_plane(candidates)
        rotation, translation, quaternion = plane_level_transform(normal, offset)
        means = np.array([stop['mean_position_m'] for stop in self.metrics['stops']])
        aligned = means @ rotation.T + translation
        timestamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%dT%H%M%S%z')
        folder = self.source_analysis.parent / (timestamp + '_B03_floor_alignment')
        folder.mkdir(exist_ok=False)
        for filename in ['metrics.json', 'trajectory.npz', 'operator_annotation.json']:
            source = self.source_analysis / filename
            if source.exists():
                shutil.copy2(str(source), str(folder / filename))
        residuals = np.abs(candidates[inliers] @ normal + offset)
        result = dict(source_analysis=str(self.source_analysis), source_run=str(self.run_directory), method='RANSAC floor points from initial 5 raw scans; evaluated stops not used for fit', diagnostic_only=True, ransac_threshold_m=0.012, normal_raw_world=normal.tolist(), offset_raw_world_m=offset, rotation=rotation.tolist(), translation_m=translation.tolist(), rotation_wxyz=quaternion.tolist(), angle_to_original_z_deg=float(np.degrees(np.arccos(normal[2]))), candidate_count=len(candidates), floor_inlier_count=int(inliers.sum()), floor_residual_p95_m=float(np.percentile(residuals, 95)), associations=associations, stop_ids=[stop['point_id'] for stop in self.metrics['stops']], raw_stop_z_m=means[:, 2].tolist(), aligned_stop_z_m=aligned[:, 2].tolist(), raw_stop_z_range_m=float(np.ptp(means[:, 2])), aligned_stop_z_range_m=float(np.ptp(aligned[:, 2])), aligned_delta_z_from_first_stop_m=(aligned[:, 2]-aligned[0, 2]).tolist(), limitations=['Floor normal comes from the same lidar and estimated poses, not independent surveyed ground truth', 'Initial scans treated as near-static; no pointwise deskew', 'Height zero is an estimated floor surface, not a surveyed reference', 'No algorithm, IMU extrinsics or original trajectory modified'])
        (folder / 'floor_alignment.json').write_text(json.dumps(result, indent=2)+'\n')
        np.save(folder / 'floor_samples_raw_world.npy', candidates[inliers][::max(1, int(inliers.sum())//4000)])
        Path('/tmp/positioning-floor-alignment-path').write_text(str(folder))
        print(json.dumps(dict(analysis_directory=str(folder), **result), indent=2))
        return folder

    def _initial_clouds(self):
        parameters = json.loads((self.run_directory / 'snapshots/ros_parameters.json').read_text())['/fastlio']['mapping']
        extrinsic_rotation = np.array(parameters['extrinsic_R']).reshape(3, 3)
        extrinsic_translation = np.array(parameters['extrinsic_T'])
        source_stamps = self.trajectory['source_stamp_s']
        first_stop = min(stop['start_unix_s'] for stop in self.metrics['stops'])
        clouds, associations = [], []
        with rosbag.Bag(str(self.run_directory / 'raw/sensors.bag')) as recording:
            for topic, message, received in recording.read_messages(topics=['/livox/lidar']):
                if received.to_sec() >= first_stop:
                    break
                if not message.points:
                    continue
                # FAST-LIO timestamps its state at scan end; CustomMsg starts the scan.
                end_stamp = message.header.stamp.to_sec() + max(point.offset_time for point in message.points)*1e-9
                index = int(np.argmin(np.abs(source_stamps-end_stamp)))
                error = float(abs(source_stamps[index]-end_stamp))
                if error > 0.06:
                    continue
                points = np.asarray([(point.x, point.y, point.z) for point in message.points], dtype=float)
                radius = np.linalg.norm(points[:, :2], axis=1)
                points = points[np.isfinite(points).all(axis=1) & (radius > 0.5) & (radius < 3) & (points[:, 2] < -0.015)]
                if not len(points):
                    continue
                rotation = quaternion_matrix(self.trajectory['rotations_wxyz'][index])
                position = self.trajectory['positions_m'][index]
                clouds.append((points @ extrinsic_rotation.T + extrinsic_translation) @ rotation.T + position)
                associations.append(dict(scan_start_stamp_s=message.header.stamp.to_sec(), pose_stamp_s=float(source_stamps[index]), matching_difference_s=error, pose_position_m=position.tolist()))
                if len(clouds) == 5:
                    break
        if len(clouds) != 5:
            raise ValueError('Need five initial scans with valid odometry before the first stop')
        if np.ptp([entry['pose_position_m'] for entry in associations], axis=0).max() > 0.04:
            raise ValueError('Initial scan segment moved too far for this static diagnostic')
        return clouds, associations


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source_analysis', type=Path)
    arguments = parser.parse_args()
    FloorAlignmentTrial(arguments.source_analysis).run()


if __name__ == '__main__':
    main()
