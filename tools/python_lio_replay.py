"""Run Python LIO baselines directly from an immutable Livox ROS1 recording."""

import argparse
from collections import deque
from dataclasses import asdict
import importlib.metadata
import json
from pathlib import Path
import struct
import time

import numpy as np
from pyquaternion import Quaternion
from rosbags.highlevel import AnyReader


class PythonLioReplay:
    """Adapt Livox timestamps, acceleration units and poses to a common IMU frame."""

    POINT_DTYPE = np.dtype([
        ('offset', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
        ('reflectivity', 'u1'), ('tag', 'u1'), ('line', 'u1'),
    ])

    def __init__(self, arguments):
        self.arguments = arguments
        self.rows = []
        self.counts = dict(lidar=0, imu=0, processed_cloud=0)
        self.pending = deque()
        self.lidar_to_imu = np.eye(4)
        self.lidar_to_imu[:3, 3] = [-.011, -.02329, .04412]

    @classmethod
    def decode_cloud(cls, raw):
        """Decode packed CustomMsg fields without allocating one object per point."""
        seconds, nanoseconds, frame_length = struct.unpack_from('<III', raw, 4)
        header_end = 16 + frame_length
        declared_count = struct.unpack_from('<I', raw, header_end + 8)[0]
        array_count = struct.unpack_from('<I', raw, header_end + 16)[0]
        if declared_count != array_count or len(raw) != header_end + 20 + 19 * array_count:
            raise ValueError('Unexpected Livox CustomMsg layout')
        packed = np.frombuffer(raw, dtype=cls.POINT_DTYPE, offset=header_end + 20)
        points = np.column_stack([packed[axis] for axis in ('x', 'y', 'z')]).astype(float)
        offsets = packed['offset'].astype(np.int64)
        if not np.isfinite(points).all() or not len(points):
            raise ValueError('Empty or nonfinite Livox scan')
        return seconds * 1_000_000_000 + nanoseconds, points, offsets

    def _initialize(self):
        if self.arguments.algorithm == 'rko':
            from rko_lio.config import LIOConfig
            from rko_lio.lio import LIO
            config = LIOConfig(max_num_threads=self.arguments.threads)
            if self.arguments.voxel_size is not None:
                config.voxel_size = self.arguments.voxel_size
            self.mapper = LIO(config)
            return asdict(config), 'rko-lio'
        from kiss_icp.config import load_config
        from kiss_icp.kiss_icp import KissICP
        config = load_config(None)
        config.registration.max_num_threads = self.arguments.threads
        self.mapper = KissICP(config)
        return config.model_dump(), 'kiss-icp'

    def _register(self, cloud):
        stamp, points, offsets = cloud
        end_stamp = stamp + int(offsets.max())
        if self.arguments.algorithm == 'rko':
            self.mapper.register_scan(points, stamp + offsets, self.lidar_to_imu)
            pose = self.mapper.pose()
        else:
            # KISS expects relative scan times normalized to [0,1]. Its pose is
            # attached to the end of the scan; convert the output origin to IMU.
            span = np.ptp(offsets)
            relative = (offsets - offsets.min()) / span if span else np.zeros(len(offsets))
            self.mapper.register_frame(points, relative)
            pose = self.mapper.last_pose @ np.linalg.inv(self.lidar_to_imu)
        quaternion = Quaternion(matrix=pose[:3, :3], atol=1e-6).elements
        self.rows.append([end_stamp / 1e9, *pose[:3, 3], *quaternion])
        self.counts['processed_cloud'] += 1

    def run(self):
        output = self.arguments.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        config, package = self._initialize()
        result = dict(status='starting', algorithm=self.arguments.algorithm,
                      source_bag=str(self.arguments.bag.resolve()), package=package,
                      package_version=importlib.metadata.version(package),
                      config=config, acceleration_scale=9.80665,
                      lidar_to_imu=self.lidar_to_imu.tolist(),
                      output_frame='imu', pose_timestamp='scan_end',
                      ground_truth_available=False)
        start = time.monotonic()
        try:
            with AnyReader([self.arguments.bag]) as reader:
                connections = [connection for connection in reader.connections
                               if connection.topic in ('/livox/lidar', '/livox/imu')]
                first_arrival = None
                for connection, arrival, raw in reader.messages(connections=connections):
                    first_arrival = arrival if first_arrival is None else first_arrival
                    if self.arguments.duration and (arrival-first_arrival)/1e9 > self.arguments.duration:
                        break
                    if connection.topic == '/livox/lidar':
                        self.counts['lidar'] += 1
                        cloud = self.decode_cloud(raw)
                        if self.arguments.algorithm == 'kiss':
                            self._register(cloud)
                        else:
                            self.pending.append(cloud)
                    else:
                        self.counts['imu'] += 1
                        if self.arguments.algorithm != 'rko':
                            continue
                        message = reader.deserialize(raw, connection.msgtype)
                        stamp = message.header.stamp.sec*1_000_000_000 + message.header.stamp.nanosec
                        acceleration = message.linear_acceleration
                        gyro = message.angular_velocity
                        self.mapper.add_imu_measurement(
                            np.array([acceleration.x, acceleration.y, acceleration.z])*9.80665,
                            np.array([gyro.x, gyro.y, gyro.z]), stamp)
                        while self.pending and self.pending[0][0]+int(self.pending[0][2].max()) <= stamp:
                            self._register(self.pending.popleft())
                    if self.counts['processed_cloud'] and self.counts['processed_cloud'] % 100 == 0 and connection.topic == '/livox/lidar':
                        print(json.dumps(dict(self.counts, elapsed_s=time.monotonic()-start)), flush=True)
                result.update(status='completed', pending_clouds_without_future_imu=len(self.pending))
                if not self.rows or not np.isfinite(self.rows).all():
                    raise ValueError('Missing or invalid trajectory')
        except BaseException as error:
            result.update(status='failed', error=str(error))
            raise
        finally:
            np.save(output/'odometry.npy', np.asarray(self.rows))
            result.update(counts=self.counts, elapsed_s=time.monotonic()-start,
                          first_stamp=self.rows[0][0] if self.rows else None,
                          last_stamp=self.rows[-1][0] if self.rows else None)
            (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--algorithm', choices=['rko', 'kiss'], required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--voxel-size', type=float,
                        help='Override RKO map voxel size in metres; otherwise retain its upstream default')
    arguments = parser.parse_args()
    if arguments.voxel_size is not None:
        if arguments.algorithm != 'rko':
            parser.error('--voxel-size is currently supported only for RKO-LIO')
        if not np.isfinite(arguments.voxel_size) or arguments.voxel_size <= 0:
            parser.error('Voxel size must be finite and positive')
    PythonLioReplay(arguments).run()


if __name__ == '__main__':
    main()
