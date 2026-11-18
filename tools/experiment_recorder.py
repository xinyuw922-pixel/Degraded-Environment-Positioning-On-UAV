"""Record MID-360/FAST-LIO2 with guided or manual timestamped stop events."""

import argparse
import csv
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import threading
import time
import uuid

import rosbag
import rospy
from nav_msgs.msg import Odometry


class ExperimentRecorder:
    """Own one new run directory and one rosbag process; never start sensors."""

    def __init__(self, arguments):
        self.arguments = arguments
        timestamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%dT%H%M%S%z')
        self.run_id = timestamp + '_B03_' + arguments.test_id + '_' + uuid.uuid4().hex[:8]
        self.directory = arguments.output_root.expanduser().resolve() / self.run_id
        self._lock = threading.Lock()
        self._process = None
        self._subscriber = None
        self._files = []
        self._active_stop = None
        self._stops = []
        self._last_odometry = None
        self._trajectory_count = 0
        self._accept_odometry = True
        self._stop_requested = False
        self._interrupted = False
        self._manifest = {}
        self._guided_round = None
        self._guided_visit = None

    def run(self):
        """Preflight, record until stop/duration, then verify and hash outputs."""
        published = dict(rospy.get_published_topics())
        required = {'/livox/lidar': 'livox_ros_driver2/CustomMsg', '/livox/imu': 'sensor_msgs/Imu'}
        if not self.arguments.raw_only:
            required[self.arguments.odom_topic] = 'nav_msgs/Odometry'
        problems = [topic for topic, expected in required.items() if published.get(topic) != expected]
        if problems:
            raise RuntimeError('Required topics absent or wrong type: ' + ', '.join(problems))
        if not sys.stdin.isatty() and self.arguments.duration is None:
            raise RuntimeError('Non-interactive recording requires --duration')
        if self.arguments.guided and not sys.stdin.isatty():
            raise RuntimeError('Guided mode requires an interactive terminal')
        if self.arguments.reference_file and not self.arguments.reference_file.expanduser().is_file():
            raise RuntimeError('Reference file does not exist')
        optional = ['/fastlio/tf', '/tf_static', '/rosout_agg']
        topics = list(required) + [topic for topic in optional if topic in published and topic not in required]
        self._prepare(topics, published)
        failure = None
        try:
            self._start(topics)
            if self.arguments.guided:
                self._guided_interact()
            else:
                self._interact()
        except KeyboardInterrupt:
            self._interrupted = True
        except Exception as error:
            failure = str(error)
        finally:
            self._finish(failure, required)
        if self._manifest['status'] != 'completed':
            raise RuntimeError('Recording not completed: ' + str(self._manifest.get('failures')))
        print('Saved: {}'.format(self.directory), flush=True)

    def _open_csv(self, relative_path, fields):
        stream = (self.directory / relative_path).open('x', newline='')
        self._files.append(stream)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        stream.flush()
        return writer

    def _prepare(self, topics, published):
        self.directory.mkdir(parents=True, exist_ok=False)
        for name in ('raw', 'reference', 'snapshots'):
            (self.directory / name).mkdir()
        self._events = self._open_csv('events.csv', ['event_id', 'run_id', 'host_unix_s', 'source_stamp_s', 'source_clock_domain', 'event_type', 'start_unix_s', 'end_unix_s', 'affected_stream', 'severity', 'description', 'action', 'exclusion_reason'])
        self._stop_writer = self._open_csv('reference/stop_observations.csv', ['run_id', 'stop_id', 'reference_id', 'point_id', 'start_unix_s', 'end_unix_s', 'clock_domain', 'rig_orientation_description', 'settled', 'operator', 'notes', 'round_index', 'visit_index'])
        self._trajectory_writer = self._open_csv('trajectory.csv', ['host_unix_s', 'source_stamp_s', 'frame_id', 'child_frame_id', 'tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw'])
        self._manifest = dict(schema_version='positioning-recording-v0.1', run_id=self.run_id, baseline_id='B03', test_id=self.arguments.test_id, purpose=self.arguments.purpose, operator=self.arguments.operator, status='prepared', result='not_evaluated', timezone='Asia/Shanghai', topics={topic: published[topic] for topic in topics}, optional_topics_missing=[topic for topic in ['/fastlio/tf', '/tf_static', '/rosout_agg'] if topic not in published], reference_file=None, reference_id=self.arguments.reference_id, reference_uncertainty_m=None, comparison_frame=None, alignment=None, continuous_ground_truth_available=False, note=self.arguments.note, failures=[], initialized=False, trajectory_is_raw_odometry=True, stop_windows_are_host_time=True)
        if self.arguments.reference_file:
            reference = self.arguments.reference_file.expanduser().resolve()
            (self.directory / 'reference/reference_points.csv').write_bytes(reference.read_bytes())
            self._manifest['reference_file'] = str(reference)
        parameters = {name: rospy.get_param(name) for name in ['/fastlio', '/use_sim_time', '/user_config_path'] if rospy.has_param(name)}
        (self.directory / 'snapshots/ros_parameters.json').write_text(json.dumps(parameters, indent=2)+'\n')
        config_path = parameters.get('/user_config_path')
        if config_path and Path(config_path).is_file():
            (self.directory / 'snapshots/livox_config.json').write_bytes(Path(config_path).read_bytes())
        project = Path(__file__).resolve().parents[1]
        revision = subprocess.run(['git', '-C', str(project), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
        self._manifest['recorder_code'] = dict(commit=revision, script_sha256=self._sha256(Path(__file__)), dirty=bool(subprocess.run(['git', '-C', str(project), 'status', '--porcelain'], capture_output=True, text=True, check=True).stdout.strip()))
        (self.directory / 'snapshots/recorder_source.py').write_bytes(Path(__file__).read_bytes())
        self._manifest['invocation'] = sys.argv
        if self.arguments.guided:
            self._manifest['guided_plan'] = dict(columns=self.arguments.grid[0], rows=self.arguments.grid[1], rounds=self.arguments.rounds, settle_seconds=self.arguments.settle_seconds, hold_seconds=self.arguments.hold_seconds, point_order='row-major P01 to P{:02d}, same order each round'.format(math.prod(self.arguments.grid)))
        self._save_manifest()

    def _start(self, topics):
        name = 'positioning_bag_' + uuid.uuid4().hex[:8]
        command = ['rosbag', 'record', '--buffsize=256', '-O', str(self.directory / 'raw/sensors.bag')] + topics + ['__name:=' + name]
        self._manifest['record_command'] = command
        self._bag_log = (self.directory / 'rosbag.log').open('x')
        self._files.append(self._bag_log)
        self._process = subprocess.Popen(command, stdout=self._bag_log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                raise RuntimeError('rosbag exited during startup; inspect rosbag.log')
            subscribers = dict(rospy.get_master().getSystemState()[2][1])
            if all('/' + name in subscribers.get(topic, []) for topic in topics):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError('rosbag subscriptions were not ready within 10 seconds')
        self._manifest.update(status='recording', recording_start_unix_s=time.time())
        if not self.arguments.raw_only:
            self._subscriber = rospy.Subscriber(self.arguments.odom_topic, Odometry, self._receive_odometry, queue_size=1000)
        self._event('recording_ready', 'Bag subscriptions ready; events use host wall time')
        self._save_manifest()
        print('RECORDING {}'.format(self.directory), flush=True)
        if not self.arguments.guided:
            print('Commands: init | start A | end | mark TEXT | stop\nStop labels are reference point IDs. Keep each stop >=10 s.', flush=True)

    def _guided_interact(self):
        """Prompt each physical placement; time settling and observation separately."""
        columns, rows = self.arguments.grid
        points = columns * rows
        total = points * self.arguments.rounds
        print('\n网格编号（每轮相同，逐行从左到右，不自动反向）：', flush=True)
        for row in range(rows):
            print('  ' + '  '.join('P{:02d}'.format(row*columns+column+1) for column in range(columns)), flush=True)
        print('共{}轮、{}次停靠。每点稳定{}秒，再记录{}秒。移动期间仍保存原始数据。'.format(self.arguments.rounds, total, self.arguments.settle_seconds, self.arguments.hold_seconds), flush=True)
        print('放稳后仅按回车；q+回车或Ctrl+C中止。不要在倒计时中移动设备。', flush=True)
        if not self._guided_confirm('确认FAST-LIO2已初始化稳定，按回车开始向导：'):
            return
        self._command('init')
        for round_index in range(1, self.arguments.rounds+1):
            self._guided_round = round_index
            self._event('round_start', 'ROUND_{}'.format(round_index))
            for point_index in range(1, points+1):
                self._guided_visit = (round_index-1)*points+point_index
                label = 'P{:02d}'.format(point_index)
                prompt = '\n[{}/{}] 第{}/{}轮，点 {}（行{}列{}）。移动到位并放稳后按回车：'.format(self._guided_visit, total, round_index, self.arguments.rounds, label, (point_index-1)//columns+1, (point_index-1)%columns+1)
                if not self._guided_confirm(prompt):
                    return
                if not self._guided_countdown('等待稳定', self.arguments.settle_seconds):
                    return
                self._command('start ' + label)
                if not self._guided_countdown('正在记录 {}，请勿移动'.format(label), self.arguments.hold_seconds):
                    return
                self._command('end')
                self._manifest['guided_progress'] = dict(completed_visits=len(self._stops), expected_visits=total)
                self._save_manifest()
                print('✓ {} 本次停靠已保存，可以移动。'.format(label), flush=True)
            self._event('round_end', 'ROUND_{}'.format(round_index))
        self._manifest['guided_sequence_complete'] = True
        self._stop_requested = True
        print('\n全部{}次停靠完成，正在关闭bag并检查数据，请等待……'.format(total), flush=True)

    def _guided_confirm(self, prompt):
        print(prompt, flush=True)
        while True:
            self._guided_check_running()
            if select.select([sys.stdin], [], [], 0.2)[0]:
                answer = sys.stdin.readline()
                if not answer or answer.strip().lower() == 'q':
                    self._interrupted = True
                    return False
                if not answer.strip():
                    return True
                print('仅按回车确认，或输入q中止。', flush=True)

    def _guided_countdown(self, label, seconds):
        deadline = time.monotonic() + seconds
        previous = None
        while time.monotonic() < deadline:
            self._guided_check_running()
            remaining = math.ceil(deadline-time.monotonic())
            if remaining != previous:
                print('\r{}：剩余 {:2d} 秒   '.format(label, remaining), end='', flush=True)
                previous = remaining
            if select.select([sys.stdin], [], [], min(0.1, max(0, deadline-time.monotonic())))[0]:
                answer = sys.stdin.readline()
                if not answer or answer.strip().lower() == 'q':
                    print('', flush=True)
                    self._interrupted = True
                    return False
        print('\r{}：完成                 '.format(label), flush=True)
        return True

    def _guided_check_running(self):
        if self._process.poll() is not None:
            raise RuntimeError('rosbag stopped unexpectedly during guided recording')
        if rospy.is_shutdown():
            raise RuntimeError('ROS shutdown during guided recording')

    def _receive_odometry(self, message):
        position, rotation = message.pose.pose.position, message.pose.pose.orientation
        with self._lock:
            if not self._accept_odometry:
                return
            self._last_odometry = message.header.stamp.to_sec()
            self._trajectory_writer.writerow(dict(host_unix_s=time.time(), source_stamp_s=self._last_odometry, frame_id=message.header.frame_id, child_frame_id=message.child_frame_id, tx=position.x, ty=position.y, tz=position.z, qx=rotation.x, qy=rotation.y, qz=rotation.z, qw=rotation.w))
            self._trajectory_count += 1

    def _event(self, kind, description):
        with self._lock:
            now = time.time()
            self._events.writerow(dict(event_id=uuid.uuid4().hex, run_id=self.run_id, host_unix_s=now, source_stamp_s=self._last_odometry, source_clock_domain='odometry_header_not_assumed_host_synchronized', event_type=kind, start_unix_s=now, end_unix_s='', affected_stream='', severity='info', description=description, action='', exclusion_reason=''))
            for stream in self._files:
                stream.flush()

    def _interact(self):
        started = time.monotonic()
        interactive = sys.stdin.isatty()
        if not interactive and self.arguments.duration is None:
            raise RuntimeError('Non-interactive recording requires --duration')
        while not self._stop_requested and not rospy.is_shutdown():
            if self._process.poll() is not None:
                raise RuntimeError('rosbag stopped unexpectedly')
            if self.arguments.duration is not None and time.monotonic() - started >= self.arguments.duration:
                break
            if interactive and select.select([sys.stdin], [], [], 0.2)[0]:
                command = sys.stdin.readline()
                if not command:
                    self._interrupted = True
                    break
                self._command(command.strip())
            else:
                time.sleep(0.1)

    def _command(self, command):
        if command == 'init':
            self._manifest.update(initialized=True, initialization_end_unix_s=time.time())
            self._event('initialization_complete', 'Operator confirmed initialization complete')
        elif command.startswith('start '):
            point = command[6:].strip()
            if self._active_stop is not None or not point or len(point.split()) != 1:
                print('End current stop first; usage: start POINT_ID', flush=True)
                return
            self._active_stop = dict(point_id=point, start_unix_s=time.time(), round_index=self._guided_round, visit_index=self._guided_visit)
            self._event('stop_start', point)
        elif command == 'end':
            self._close_stop(complete=True)
        elif command.startswith('mark '):
            self._event('operator_note', command[5:])
        elif command == 'stop':
            self._stop_requested = True
        else:
            print('Use: init | start A | end | mark TEXT | stop', flush=True)

    def _close_stop(self, complete):
        if self._active_stop is None:
            return
        stop = self._active_stop
        ended = time.time()
        self._stop_writer.writerow(dict(run_id=self.run_id, stop_id=uuid.uuid4().hex, reference_id=self.arguments.reference_id, point_id=stop['point_id'], start_unix_s=stop['start_unix_s'], end_unix_s=ended, clock_domain='host_unix', rig_orientation_description='', settled='', operator=self.arguments.operator, notes='' if complete else 'incomplete: recording ended before explicit end', round_index=stop['round_index'], visit_index=stop['visit_index']))
        self._stops.append(dict(point_id=stop['point_id'], duration_s=ended-stop['start_unix_s'], complete=complete, round_index=stop['round_index'], visit_index=stop['visit_index']))
        self._event('stop_end' if complete else 'stop_incomplete', stop['point_id'])
        self._active_stop = None

    def _finish(self, failure, required):
        self._close_stop(complete=False)
        with self._lock:
            self._accept_odometry = False
        if self._subscriber is not None:
            self._subscriber.unregister()
        forced = False
        if self._process is not None and self._process.poll() is None:
            os.killpg(self._process.pid, signal.SIGINT)
            try:
                self._process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                forced = True
                os.killpg(self._process.pid, signal.SIGTERM)
                try:
                    self._process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(self._process.pid, signal.SIGKILL)
                    self._process.wait()
        self._manifest.update(recording_end_unix_s=time.time(), trajectory_rows=self._trajectory_count, stops=self._stops, interrupted=self._interrupted, forced_shutdown=forced, rosbag_exit_code=self._process.returncode if self._process else None)
        if failure:
            self._manifest['failures'].append(failure)
        bag_path = self.directory / 'raw/sensors.bag'
        try:
            with rosbag.Bag(str(bag_path), 'r') as recording:
                information = recording.get_type_and_topic_info().topics
                self._manifest['bag_topics'] = {topic: dict(type=detail.msg_type, messages=detail.message_count, frequency_hz=detail.frequency) for topic, detail in information.items()}
                for topic in required:
                    if topic not in information or information[topic].message_count == 0:
                        self._manifest['failures'].append('Missing recorded stream: ' + topic)
        except Exception as error:
            self._manifest['failures'].append('Bag verification: ' + str(error))
        if forced or (self.directory / 'raw/sensors.bag.active').exists():
            self._manifest['failures'].append('Bag finalization incomplete')
        if self._process and self._process.returncode not in (0, -signal.SIGINT):
            self._manifest['failures'].append('rosbag exit code: ' + str(self._process.returncode))
        self._manifest['status'] = 'failed' if self._manifest['failures'] else ('aborted' if self._interrupted else 'completed')
        with self._lock:
            for stream in self._files:
                stream.close()
        checksums = {str(path.relative_to(self.directory)): dict(sha256=self._sha256(path), bytes=path.stat().st_size) for path in self.directory.rglob('*') if path.is_file() and path.name not in ('manifest.json', 'checksums.json')}
        (self.directory / 'checksums.json').write_text(json.dumps(checksums, indent=2)+'\n')
        self._save_manifest()

    def _save_manifest(self):
        temporary = self.directory / 'manifest.tmp'
        temporary.write_text(json.dumps(self._manifest, indent=2)+'\n')
        temporary.replace(self.directory / 'manifest.json')

    @staticmethod
    def _sha256(path):
        digest = hashlib.sha256()
        with path.open('rb') as stream:
            for block in iter(lambda: stream.read(1024*1024), b''):
                digest.update(block)
        return digest.hexdigest()


def main():
    """Parse recording options; services must already be running."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root', type=Path, default=Path.home() / 'positioning_data' / 'runs')
    parser.add_argument('--test-id')
    parser.add_argument('--purpose', choices=['calibration', 'mapping', 'tuning', 'acceptance'])
    parser.add_argument('--operator', required=True)
    parser.add_argument('--odom-topic', default='/fastlio/Odometry')
    parser.add_argument('--reference-id', default='unregistered')
    parser.add_argument('--reference-file', type=Path)
    parser.add_argument('--note', default='')
    parser.add_argument('--raw-only', action='store_true')
    parser.add_argument('--duration', type=float)
    parser.add_argument('--guided', action='store_true', help='Prompt each grid point and automatically time stops')
    parser.add_argument('--grid', type=int, nargs=2, default=[4, 3], metavar=('COLUMNS', 'ROWS'))
    parser.add_argument('--rounds', type=int, default=2)
    parser.add_argument('--settle-seconds', type=float, default=5)
    parser.add_argument('--hold-seconds', type=float, default=10)
    arguments = parser.parse_args()
    if arguments.guided:
        arguments.test_id = arguments.test_id or 'GRID_{}x{}'.format(*arguments.grid)
        arguments.purpose = arguments.purpose or 'tuning'
        if arguments.duration is not None or arguments.raw_only:
            parser.error('--guided cannot be combined with --duration or --raw-only')
    elif not arguments.test_id or not arguments.purpose:
        parser.error('Manual mode requires --test-id and --purpose')
    if min(arguments.grid) < 1 or arguments.rounds < 1:
        parser.error('Grid dimensions and rounds must be positive')
    if not math.isfinite(arguments.settle_seconds) or arguments.settle_seconds < 0 or not math.isfinite(arguments.hold_seconds) or arguments.hold_seconds < 10:
        parser.error('Settling must be nonnegative; hold time must be at least 10 seconds')
    if not arguments.test_id.replace('-', '').replace('_', '').isalnum():
        parser.error('--test-id must contain letters/numbers, hyphens or underscores')
    if arguments.duration is not None and arguments.duration <= 0:
        parser.error('--duration must be positive')
    rospy.init_node('positioning_record_monitor', anonymous=True, disable_signals=True)
    try:
        ExperimentRecorder(arguments).run()
    except (RuntimeError, OSError, rospy.ROSException) as error:
        print('Recording error: {}'.format(error), file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
