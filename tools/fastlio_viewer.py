"""Display live FAST-LIO2 registered clouds and odometry in viser."""

import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import viser

from utils.pointcloud_utils import xyz_points


class FastlioViewer:
    """Keep only the latest scan so port forwarding cannot build a frame backlog."""

    def __init__(self, port, status_path):
        self._lock = threading.Lock()
        self._cloud = None
        self._odometry = None
        self._counts = {'cloud': 0, 'odometry': 0}
        self._received = {}
        self._trajectory = deque(maxlen=3000)
        self._map_points = np.empty((0, 3), dtype=np.float32)
        self._last_map_update = 0
        self._status_path = status_path
        self._server = viser.ViserServer(host='127.0.0.1', port=port, label='FAST-LIO2 Live')
        self._server.scene.set_up_direction('+z')
        self._server.scene.add_grid('/grid', width=20, height=20, plane='xy')
        self._body = self._server.scene.add_frame('/body', axes_length=0.4, axes_radius=0.01)
        self._scan_handle = None
        self._map_handle = None
        self._trajectory_handle = None
        self._pause = self._server.gui.add_checkbox('Pause display', initial_value=False)
        self._show_map = self._server.gui.add_checkbox('Show accumulated preview', initial_value=True)
        self._point_limit = self._server.gui.add_slider('Scan points', min=2000, max=20000, step=1000, initial_value=10000)
        self._status = self._server.gui.add_markdown('Waiting for FAST-LIO2...')
        self._server.gui.add_markdown('Orange: current scan · blue: accumulated preview · green: trajectory.\n\nLocal start frame, metres. Preview is bounded and downsampled; it is not the saved algorithm map. Drag to orbit, scroll to zoom, right drag to pan.')
        self._server.on_client_connect(self._set_camera)
        self._subscribers = [
            rospy.Subscriber('/fastlio/cloud_registered', PointCloud2, self._receive_cloud, queue_size=1, buff_size=8*1024*1024),
            rospy.Subscriber('/fastlio/Odometry', Odometry, self._receive_odometry, queue_size=1),
        ]

    def _set_camera(self, client):
        client.camera.position = (4, -6, 4)
        client.camera.look_at = (0, 0, 0.5)
        client.camera.up_direction = (0, 0, 1)

    def _receive_cloud(self, message):
        with self._lock:
            self._cloud = message
            self._counts['cloud'] += 1
            self._received['cloud'] = time.monotonic()

    def _receive_odometry(self, message):
        position = message.pose.pose.position
        coordinates = (position.x, position.y, position.z)
        if not np.isfinite(coordinates).all():
            return
        with self._lock:
            self._odometry = message
            self._counts['odometry'] += 1
            self._received['odometry'] = time.monotonic()
            if not self._trajectory or np.linalg.norm(np.subtract(coordinates, self._trajectory[-1])) > 0.02:
                self._trajectory.append(coordinates)

    def _render_scan(self, message):
        points = xyz_points(message, int(self._point_limit.value))
        if self._scan_handle is None:
            self._scan_handle = self._server.scene.add_point_cloud('/scan', points=points, colors=(255, 170, 60), point_size=0.025)
        else:
            self._scan_handle.points = points
        if time.monotonic() - self._last_map_update < 1:
            return
        # Bound visual history independently of the algorithm's internal map.
        combined = np.concatenate((self._map_points, points))
        if len(combined):
            _, indices = np.unique(np.floor(combined / 0.10).astype(np.int32), axis=0, return_index=True)
            self._map_points = combined[np.sort(indices)][-40000:]
        if self._map_handle is None:
            self._map_handle = self._server.scene.add_point_cloud('/map_preview', points=self._map_points, colors=(90, 170, 230), point_size=0.018)
        else:
            self._map_handle.points = self._map_points
        self._last_map_update = time.monotonic()

    def _render_pose(self, odometry, trajectory):
        position = odometry.pose.pose.position
        rotation = odometry.pose.pose.orientation
        self._body.position = (position.x, position.y, position.z)
        self._body.wxyz = (rotation.w, rotation.x, rotation.y, rotation.z)
        if len(trajectory) > 1:
            coordinates = np.asarray(trajectory, dtype=np.float32)
            segments = np.stack((coordinates[:-1], coordinates[1:]), axis=1)
            if self._trajectory_handle is None:
                self._trajectory_handle = self._server.scene.add_line_segments('/trajectory', points=segments, colors=(70, 240, 110), line_width=3)
            else:
                self._trajectory_handle.points = segments

    def run(self):
        """Serve at up to 8 FPS; write receive health once per second."""
        last_cloud = None
        last_status = 0
        previous_counts = {'cloud': 0, 'odometry': 0}
        try:
            while not rospy.is_shutdown():
                started = time.monotonic()
                with self._lock:
                    cloud, odometry = self._cloud, self._odometry
                    counts, received = self._counts.copy(), self._received.copy()
                    trajectory = list(self._trajectory)
                if not self._pause.value:
                    if cloud is not None and cloud is not last_cloud:
                        self._render_scan(cloud)
                        last_cloud = cloud
                    if odometry is not None:
                        self._render_pose(odometry, trajectory)
                if self._map_handle is not None:
                    self._map_handle.visible = self._show_map.value
                if started - last_status >= 1:
                    interval = started - last_status if last_status else 1
                    rates = {name: round((count-previous_counts[name])/interval, 1) for name, count in counts.items()}
                    ages = {name: round(started-arrived, 3) for name, arrived in received.items()}
                    live = all(ages.get(name, 999) < 1 for name in counts)
                    position = list(self._body.position)
                    self._status.content = '**{}**\n\nCloud: {} Hz · Pose: {} Hz\n\nPosition (m): {:.3f}, {:.3f}, {:.3f}\n\nDisplay ≤8 FPS · Clients: {}'.format('LIVE' if live else 'Waiting / stale', rates['cloud'], rates['odometry'], *position, len(self._server.get_clients()))
                    report = dict(live=live, counts=counts, hz=rates, receive_age_s=ages, position_m=position, map_preview_points=len(self._map_points), clients=len(self._server.get_clients()), frame=cloud.header.frame_id if cloud else None, updated_unix=time.time())
                    self._status_path.parent.mkdir(parents=True, exist_ok=True)
                    temporary = self._status_path.with_suffix('.tmp')
                    temporary.write_text(json.dumps(report, indent=2)+'\n')
                    temporary.replace(self._status_path)
                    last_status, previous_counts = started, counts
                time.sleep(max(0.005, 0.125-(time.monotonic()-started)))
        finally:
            for subscriber in self._subscribers:
                subscriber.unregister()
            self._server.stop()


def main():
    """Start a ROS subscriber and a localhost viser server."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--status-path', type=Path, required=True)
    arguments = parser.parse_args()
    rospy.init_node('fastlio_viser', anonymous=False, disable_signals=True)
    try:
        FastlioViewer(arguments.port, arguments.status_path).run()
    except KeyboardInterrupt:
        rospy.signal_shutdown('Viewer stopped')


if __name__ == '__main__':
    main()
