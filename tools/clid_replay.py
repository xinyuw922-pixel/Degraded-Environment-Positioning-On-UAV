"""Run upstream CLID-SLAM in an isolated, patched runtime source copy."""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import time

import numpy as np
import yaml


class ClidReplay:
    """Preserve defaults while adapting MID360 extrinsics, units and CPU support."""

    def __init__(self, arguments):
        self.arguments = arguments

    def run(self):
        arguments = self.arguments
        output = arguments.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        source = output/'source'
        shutil.copytree(arguments.upstream.resolve(), source)
        patch = Path(__file__).resolve().parents[1]/'third_party/patches/clid_cpu_and_imu_csv.patch'
        with (output/'patch.log').open('x') as log:
            subprocess.run(['patch', '-p1', '-i', str(patch)], cwd=source,
                           stdout=log, stderr=subprocess.STDOUT, check=True)
        for changed in ("utils/slam_dataset.py", "utils/error_state_iekf.py", "utils/tools.py", "slam.py"):
            compile((source/changed).read_text(), str(source/changed), "exec")
        dataset = arguments.dataset.resolve()
        config = yaml.safe_load((source/'config/run_ncd128.yaml').read_text())
        config['setting'].update(name='mid360_cpu', output_root=str(output/'runs'),
            pc_path=str(dataset/'lidar'), imu_path=str(dataset/'imu'),
            pose_ts_path=str(dataset/'pose_ts.txt'), device='cpu')
        if arguments.frames:
            config['setting']['end_frame'] = arguments.frames
        config['tracker']['T_imu_lidar'] = [
            [1., 0., 0., -.011], [0., 1., 0., -.02329],
            [0., 0., 1., .04412], [0., 0., 0., 1.]]
        config['eval'].update(o3d_vis_on=False, wandb_vis_on=False,
            save_map=False, save_mesh=False, save_merged_pc=False,
            mesh_freq_frame=1000000)
        parameters = output/'effective_params.yaml'
        parameters.write_text(yaml.safe_dump(config))
        command = [str(arguments.python.absolute()), '-u', str(source/'slam.py'), str(parameters)]
        result = dict(status='running', algorithm='CLID-SLAM', device='cpu',
            dataset_manifest=json.loads((dataset/'manifest.json').read_text()),
            command=command, output_frame='imu', pose_timestamp='scan midpoint',
            requested_frames=arguments.frames, cpu_threads=arguments.threads, ground_truth_available=False)
        result_path = output/'result.json'
        result_path.write_text(json.dumps(result, indent=2)+'\n')
        started = time.monotonic()
        try:
            environment = os.environ.copy()
            environment.pop('PYTHONPATH', None)
            environment['CLID_CPU_THREADS'] = str(arguments.threads)
            registry = yaml.safe_load((Path(__file__).resolve().parents[1]/'third_party/sources.yaml').read_text())
            environment['CLID_SOURCE_COMMIT'] = next(entry['commit'] for entry in registry['dependencies'] if entry['name'] == 'clid_slam')
            with (output/'mapper.log').open('x') as log:
                subprocess.run(command, cwd=source, stdout=log,
                               stderr=subprocess.STDOUT, env=environment, check=True)
            trajectories = list((output/'runs').glob('*/odom_poses_tum.txt'))
            if len(trajectories) != 1:
                raise ValueError('Expected one upstream odom_poses_tum.txt trajectory')
            poses = np.loadtxt(trajectories[0])
            if poses.ndim != 2 or poses.shape[1] != 8 or not np.isfinite(poses).all():
                raise ValueError('Invalid CLID trajectory')
            expected = arguments.frames or result['dataset_manifest']['scans']
            if len(poses) != expected:
                raise ValueError('CLID trajectory does not cover all requested frames')
            if np.any(np.diff(poses[:, 0]) <= 0):
                raise ValueError('CLID timestamps must increase')
            np.save(output/'odometry.npy', poses[:, [0, 1, 2, 3, 7, 4, 5, 6]])
            result.update(status='completed', odometry_messages=len(poses),
                          first_stamp=poses[0, 0], last_stamp=poses[-1, 0])
        except BaseException as error:
            result.update(status='failed', error=str(error))
            raise
        finally:
            result['elapsed_s'] = time.monotonic()-started
            result_path.write_text(json.dumps(result, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--python', type=Path, required=True)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--dataset', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int)
    parser.add_argument('--threads', type=int, default=16)
    ClidReplay(parser.parse_args()).run()


if __name__ == '__main__':
    main()
