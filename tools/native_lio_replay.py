"""Configure and run native headless Traj-LO or GLIM on a recorded bag."""

import argparse
import json
from pathlib import Path
import re
import shutil
import subprocess
import time

import numpy as np
import yaml


class NativeLioReplay:
    """Keep runtime configuration, complete logs and a standard IMU trajectory."""

    def __init__(self, arguments):
        self.arguments = arguments
        self.output = arguments.output.resolve()

    def _configure(self):
        source = self.arguments.upstream.resolve()
        if self.arguments.algorithm == 'trajlo':
            config = yaml.safe_load((source/'data/config_livox.yaml').read_text())
            config['dataset'].update(path=str(self.arguments.bag.resolve()),
                                     pose_file_path=str(self.output/'poses.txt'))
            config['calibration']['T_body_lidar'] = [
                1., 0., 0., -.011, 0., 1., 0., -.02329,
                0., 0., 1., .04412, 0., 0., 0., 1.]
            path = self.output/'effective_params.yaml'
            path.write_text(yaml.safe_dump(config))
            return [str(path)]
        config_directory = self.output/'config'
        shutil.copytree(source/'config', config_directory)
        for filename, module, changes in [
            ('config.json', 'global', {'config_odometry': 'config_odometry_cpu.json'}),
            ('config_sensors.json', 'sensors', {
                'T_lidar_imu': [.011, .02329, -.04412, 0., 0., 0., 1.],
                'global_shutter_lidar': False}),
        ]:
            path = config_directory/filename
            text = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.S)
            config = json.loads(text)
            config[module].update(changes)
            path.write_text(json.dumps(config, indent=2)+'\n')
        return [str(config_directory), str(self.arguments.bag.resolve()),
                str(self.output/'poses.txt')]

    def run(self):
        self.output.mkdir(parents=True, exist_ok=False)
        command = [str(self.arguments.binary.resolve()), *self._configure()]
        result = dict(status='starting', algorithm=self.arguments.algorithm,
                      source_bag=str(self.arguments.bag.resolve()), command=command,
                      output_frame='imu', ground_truth_available=False,
                      global_loop_closure=False)
        started = time.monotonic()
        try:
            with (self.output/'mapper.log').open('x') as log:
                subprocess.run(command, stdout=log, stderr=subprocess.STDOUT, check=True)
            poses = np.loadtxt(self.output/'poses.txt')
            if poses.ndim != 2 or poses.shape[1] != 8 or not np.isfinite(poses).all():
                raise ValueError('Missing or invalid native trajectory')
            if np.any(np.diff(poses[:, 0]) <= 0):
                raise ValueError('Native output timestamps must increase')
            np.save(self.output/'odometry.npy', poses[:, [0, 1, 2, 3, 7, 4, 5, 6]])
            log_text = (self.output/'mapper.log').read_text()
            counts = re.search(r'Completed lidar=(\d+) imu=(\d+)', log_text)
            if counts:
                result['counts'] = dict(lidar=int(counts[1]), imu=int(counts[2]))
            result.update(status='completed', odometry_messages=len(poses),
                          first_stamp=poses[0, 0], last_stamp=poses[-1, 0])
        except BaseException as error:
            result.update(status='failed', error=str(error))
            raise
        finally:
            result['elapsed_s'] = time.monotonic()-started
            (self.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithm', choices=['trajlo', 'glim'], required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--upstream', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    NativeLioReplay(parser.parse_args()).run()


if __name__ == '__main__':
    main()
