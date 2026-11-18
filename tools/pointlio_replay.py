"""Replay raw Livox data into an isolated, prebuilt LIO baseline."""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, PointCloud2
import yaml


class OfflineLioReplay:
    """Own a dedicated ROS master, player and mapper; preserve all source data."""

    def __init__(self, arguments):
        self.arguments = arguments
        self.directory = arguments.output.resolve()
        self.processes = []
        self.logs = []
        self.lock = threading.Lock()
        self.rows = []
        self.counts = dict(lidar=0, imu=0, odometry=0, invalid_odometry=0)
        if arguments.algorithm == 'dlio':
            self.counts['processed_cloud'] = 0
        self.first_stamp = None
        self.last_stamp = None

    def _start(self, name, command):
        log = (self.directory / (name+'.log')).open('x')
        self.logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        return process

    def _receive(self, message, kind):
        with self.lock:
            self.counts[kind] += 1
            if kind == 'odometry':
                position, rotation = message.pose.pose.position, message.pose.pose.orientation
                values = [position.x, position.y, position.z, rotation.w, rotation.x, rotation.y, rotation.z]
                if not all(math.isfinite(value) for value in values):
                    self.counts['invalid_odometry'] += 1
                stamp = message.header.stamp.to_sec()
                self.rows.append([stamp]+values)
                self.first_stamp = self.first_stamp or stamp
                self.last_stamp = stamp

    def _save_progress(self):
        with self.lock:
            result = dict(counts=self.counts.copy(), first_stamp=self.first_stamp, last_stamp=self.last_stamp, span_s=self.last_stamp-self.first_stamp if self.first_stamp else 0)
        (self.directory / 'progress.json').write_text(json.dumps(result, indent=2)+'\n')
        return result

    def run(self):
        """Run one closed replay and save finite-state and completion evidence."""
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(('127.0.0.1', self.arguments.master_port))
        self.directory.mkdir(parents=True, exist_ok=False)
        runtime_logs = self.directory / 'algorithm_logs'
        runtime_logs.mkdir()
        if self.arguments.algorithm in ('fastlio', 'pointlio'):
            source_package = 'FAST_LIO' if self.arguments.algorithm == 'fastlio' else 'point_lio'
            compiled_logs = self.arguments.workspace / ('src/'+source_package+'/Log')
            if compiled_logs.is_symlink():
                compiled_logs.unlink()
            elif compiled_logs.exists():
                compiled_logs.rename(self.directory / 'prior_source_logs')
            compiled_logs.symlink_to(runtime_logs, target_is_directory=True)
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:{}'.format(self.arguments.master_port), ROS_HOSTNAME='127.0.0.1', ROS_LOG_DIR=str(self.directory/'ros_logs'))
        os.environ.pop('ROS_IP', None)
        result = dict(status='starting', algorithm=self.arguments.algorithm, replay_rate=self.arguments.rate, requested_duration_s=self.arguments.duration, source_bag=str(self.arguments.bag), ground_truth_available=False)
        subscribers = []
        try:
            self._start('master', ['roscore', '-p', str(self.arguments.master_port)])
            master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            for attempt in range(150):
                try:
                    master.getUri('/lio_runner')
                    break
                except OSError:
                    time.sleep(0.1)
            else:
                raise RuntimeError('Isolated ROS master failed to start')
            rospy.init_node('lio_observer', anonymous=True, disable_signals=True)
            parameters = yaml.safe_load(self.arguments.parameters.read_text())
            if self.arguments.algorithm in ('fastlio', 'pointlio'):
                parameters['common']['lid_topic'] = '/replay/lidar'
                parameters['common']['imu_topic'] = '/replay/imu'
                parameters['publish']['scan_publish_en'] = False
            if self.arguments.algorithm == 'superlio':
                parameters['lio']['ros'].update(lidar_topic='/replay/lidar', imu_topic='/replay/imu')
                parameters['lio']['map']['save_map'] = False
                parameters['lio']['output'].update(map=False, dense=False)
            rospy.set_param('/use_sim_time', True)
            parameter_prefix = {'fastlio':'/fastlio', 'pointlio':'/pointlio', 'dlio':'/dlio', 'superlio':''}[self.arguments.algorithm]
            for key, value in parameters.items():
                rospy.set_param(parameter_prefix+'/'+key, value)
            (self.directory / 'effective_params.yaml').write_text(yaml.safe_dump(parameters))
            subscribers = [rospy.Subscriber(topic, kind, self._receive, callback_args=label, queue_size=10000, buff_size=16*1024*1024) for topic, kind, label in [('/replay/lidar', CustomMsg, 'lidar'), ('/replay/imu', Imu, 'imu'), ('/pointlio/Odometry', Odometry, 'odometry')]]
            if self.arguments.algorithm == 'dlio':
                subscribers.append(rospy.Subscriber('/dlio/deskewed', PointCloud2, self._receive, callback_args='processed_cloud', queue_size=1000, buff_size=16*1024*1024))
            if self.arguments.algorithm == 'fastlio':
                mapper_command = [str(self.arguments.workspace/'devel/lib/fast_lio/fastlio_mapping'), '__name:=mapping', '__ns:=/fastlio', '/Odometry:=/pointlio/Odometry']
            elif self.arguments.algorithm == 'superlio':
                mapper_command = [str(self.arguments.workspace/'devel/lib/super_lio/super_lio_node'), '/lio/odom:=/pointlio/Odometry']
            elif self.arguments.algorithm == 'dlio':
                mapper_command = [str(self.arguments.workspace/'devel/lib/direct_lidar_inertial_odometry/dlio_odom_node'), '__name:=dlio', '/dlio/imu:=/replay/imu', '/dlio/livox:=/replay/lidar', '/dlio/pointcloud:=/dlio/converted', '/dlio/livox2dlio:=/dlio/converted', '/dlio/odom:=/pointlio/Odometry']
            else:
                mapper_command = [str(self.arguments.workspace/'devel/lib/point_lio/pointlio_mapping'), '__name:=pointlio', '/aft_mapped_to_init:=/pointlio/Odometry']
            mapper = self._start('mapper', mapper_command)
            time.sleep(1)
            command = ['rosbag', 'play', str(self.arguments.bag), '--clock', '--rate='+str(self.arguments.rate), '--delay=1', '--topics', '/livox/lidar', '/livox/imu', '/livox/lidar:=/replay/lidar', '/livox/imu:=/replay/imu']
            if self.arguments.duration is not None:
                command += ['--duration='+str(self.arguments.duration)]
            (self.directory / 'replay_command.json').write_text(json.dumps(command, indent=2)+'\n')
            player = self._start('player', command)
            next_report = time.monotonic()
            while player.poll() is None:
                if mapper.poll() is not None:
                    raise RuntimeError('Mapper exited: '+str(mapper.returncode))
                if time.monotonic() >= next_report:
                    print(json.dumps(self._save_progress()), flush=True)
                    next_report = time.monotonic()+15
                time.sleep(.25)
            time.sleep(2)
            result.update(status='completed', player_exit_code=player.returncode, mapper_exit_before_shutdown=mapper.poll(), **self._save_progress())
            if player.returncode != 0 or mapper.poll() is not None or not self.rows or self.counts['invalid_odometry']:
                raise RuntimeError('Replay did not pass finite-output and completion checks')
        except BaseException as error:
            result.update(status='failed', error=str(error), **self._save_progress())
            raise
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
            with self.lock:
                rows = np.asarray(self.rows, dtype=float)
            np.save(self.directory/'odometry.npy', rows)
            for process in reversed(self.processes):
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGTERM)
                        process.wait(timeout=5)
            for log in self.logs:
                log.close()
            (self.directory/'result.json').write_text(json.dumps(result, indent=2)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', type=Path, required=True)
    parser.add_argument('--parameters', type=Path, required=True)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--master-port', type=int, default=11341)
    parser.add_argument('--rate', type=float, default=1)
    parser.add_argument('--duration', type=float)
    parser.add_argument('--algorithm', choices=['pointlio', 'fastlio', 'dlio', 'superlio'], default='pointlio')
    arguments = parser.parse_args()
    if not math.isfinite(arguments.rate) or arguments.rate <= 0:
        parser.error('Rate must be finite and positive')
    OfflineLioReplay(arguments).run()


if __name__ == '__main__':
    main()
