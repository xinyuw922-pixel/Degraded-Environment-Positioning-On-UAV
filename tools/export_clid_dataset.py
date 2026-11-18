"""Export Livox scans and source-time IMU intervals for upstream CLID-SLAM."""

import argparse
import json
from pathlib import Path

import numpy as np
from plyfile import PlyData, PlyElement
from rosbags.highlevel import AnyReader

from tools.python_lio_replay import PythonLioReplay


class ClidDatasetExporter:
    """Preserve scan contents and use midpoint pose times for upstream deskewing."""

    def __init__(self, arguments):
        self.arguments = arguments

    def run(self):
        output = self.arguments.output.resolve()
        output.mkdir(parents=True, exist_ok=False)
        for directory in ('lidar', 'imu'):
            (output/directory).mkdir()
        with AnyReader([self.arguments.bag]) as reader:
            imu_rows = []
            imu_connections = [c for c in reader.connections if c.topic == '/livox/imu']
            for connection, _, raw in reader.messages(connections=imu_connections):
                message = reader.deserialize(raw, connection.msgtype)
                stamp = message.header.stamp.sec*1e9 + message.header.stamp.nanosec
                acceleration, gyro = message.linear_acceleration, message.angular_velocity
                imu_rows.append([stamp/1e9, acceleration.x*9.80665,
                                 acceleration.y*9.80665, acceleration.z*9.80665,
                                 gyro.x, gyro.y, gyro.z])
            imu = np.asarray(imu_rows)
            if np.any(np.diff(imu[:, 0]) <= 0):
                raise ValueError('IMU timestamps must be increasing')
            pose_stamps = []
            previous_stamp = None
            lidar_connections = [c for c in reader.connections if c.topic == '/livox/lidar']
            for _, _, raw in reader.messages(connections=lidar_connections):
                stamp, points, offsets = PythonLioReplay.decode_cloud(raw)
                midpoint = (stamp + (int(offsets.min())+int(offsets.max()))/2)/1e9
                pose_stamps.append(midpoint)
                index = len(pose_stamps)-1
                vertex = np.empty(len(points), dtype=[('x', '<f4'), ('y', '<f4'),
                    ('z', '<f4'), ('intensity', '<f4'), ('timestamp', '<f8')])
                for axis, name in enumerate(('x', 'y', 'z')):
                    vertex[name] = points[:, axis]
                packed = np.frombuffer(raw, dtype=PythonLioReplay.POINT_DTYPE,
                                       offset=len(raw)-19*len(points))
                vertex['intensity'] = packed['reflectivity']
                vertex['timestamp'] = offsets/1e9
                PlyData([PlyElement.describe(vertex, 'vertex')], text=False).write(
                    str(output/'lidar'/('{:06d}.ply'.format(index))))
                interval = imu[(imu[:, 0] > (previous_stamp or midpoint)) & (imu[:, 0] <= midpoint)].copy()
                if len(interval):
                    interval[:, 0] = np.diff(np.r_[previous_stamp, interval[:, 0]])
                    # Integrate the final partial IMU interval to the pose time.
                    remaining = midpoint - imu[imu[:, 0] <= midpoint][-1, 0]
                    if remaining > 0:
                        interval = np.vstack([interval, np.r_[remaining, interval[-1, 1:]]])
                np.savetxt(output/'imu'/(str(index)+'.csv'), interval,
                           delimiter=',', header='dt,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z', comments='')
                previous_stamp = midpoint
                if self.arguments.frames and len(pose_stamps) >= self.arguments.frames:
                    break
                if len(pose_stamps) % 500 == 0:
                    print('Exported {} scans'.format(len(pose_stamps)), flush=True)
        np.savetxt(output/'pose_ts.txt', pose_stamps, delimiter=',', header='timestamp', comments='')
        (output/'manifest.json').write_text(json.dumps(dict(
            source_bag=str(self.arguments.bag), scans=len(pose_stamps), imu_messages=len(imu),
            point_time_unit='seconds relative to scan header', pose_time='scan midpoint',
            acceleration_scale=9.80665, original_recording_modified=False), indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--frames', type=int)
    ClidDatasetExporter(parser.parse_args()).run()


if __name__ == '__main__':
    main()
