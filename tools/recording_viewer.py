"""Serve a recorded trajectory and sparse stop statistics in viser."""

import argparse
import json
from pathlib import Path
import time

import numpy as np
import viser


class RecordingViewer:
    """Display preserved raw-frame positions, with a playback cursor."""

    def __init__(self, directory, port):
        self.directory = directory
        self.metrics = json.loads((directory / 'metrics.json').read_text())
        trajectory = np.load(directory / 'trajectory.npz')
        self.positions = trajectory['positions_m']
        self.rotations = trajectory['rotations_wxyz']
        self.times = trajectory['arrival_unix_s']
        alignment_path = directory / 'floor_alignment.json'
        self.alignment = json.loads(alignment_path.read_text()) if alignment_path.exists() else None
        self.server = viser.ViserServer(host='127.0.0.1', port=port, label='FAST-LIO2 · Recorded test')
        self.server.scene.set_up_direction('+z')
        self.server.scene.add_grid('/grid', width=2, height=2, cell_size=0.1, cell_thickness=0.5, section_size=0.5, plane='xy')
        self.scene_frame = self.server.scene.add_frame('/recording', show_axes=False)
        self.align_view = self.server.gui.add_checkbox('Floor-aligned view (diagnostic)', initial_value=self.alignment is not None, disabled=self.alignment is None)
        self.floor_handle = None
        self.show_floor = self.server.gui.add_checkbox('Show initial floor samples', initial_value=False, disabled=self.alignment is None)
        if self.alignment is not None:
            self.scene_frame.wxyz = tuple(self.alignment['rotation_wxyz'])
            self.scene_frame.position = tuple(self.alignment['translation_m'])
            floor = np.load(directory / 'floor_samples_raw_world.npy').astype(np.float32)
            self.floor_handle = self.server.scene.add_point_cloud('/recording/floor', points=floor, colors=(140, 140, 160), point_size=0.012, visible=False)
            outcome = 'smaller' if self.alignment['aligned_stop_z_range_m'] < self.alignment['raw_stop_z_range_m'] else 'larger'
            self.server.gui.add_markdown('**Floor alignment trial**\n\nRotation from initial floor echoes, without fitting A/B/C/D.\n\nStop Z range: raw **{:.2f} cm**, aligned **{:.2f} cm** ({}). Original odometry is preserved. This is a diagnostic, not independent accuracy validation.'.format(100*self.alignment['raw_stop_z_range_m'], 100*self.alignment['aligned_stop_z_range_m'], outcome))
            lines = ['| Stop | Raw ΔZ from first stop (cm) | Aligned ΔZ (cm) |', '|---|---:|---:|']
            first = self.alignment['raw_stop_z_m'][0]
            for label, raw, aligned in zip(self.alignment['stop_ids'], self.alignment['raw_stop_z_m'], self.alignment['aligned_delta_z_from_first_stop_m']):
                lines.append('| {} | {:.2f} | {:.2f} |'.format(label, 100*(raw-first), 100*aligned))
            self.server.gui.add_markdown('\n'.join(lines))
        segments = np.stack((self.positions[:-1], self.positions[1:]), axis=1).astype(np.float32)
        self.server.scene.add_line_segments('/recording/trajectory', points=segments, colors=(60, 220, 190), line_width=2)
        reference_path = directory / 'reference_trajectory.npz'
        if reference_path.exists():
            reference = np.load(reference_path)['positions_m']
            reference_segments = np.stack((reference[:-1], reference[1:]), axis=1).astype(np.float32)
            self.server.scene.add_line_segments('/recording/reference_trajectory', points=reference_segments, colors=(190, 100, 255), line_width=2)
        additional_path = directory / 'additional_trajectories.npz'
        if additional_path.exists():
            additional = np.load(additional_path)
            color_path = directory / 'additional_trajectories.json'
            colors = json.loads(color_path.read_text()) if color_path.exists() else {}
            for name in additional.files:
                positions = additional[name]
                self._add_overlay(name, positions, tuple(colors.get(name, (255, 180, 60))))
        self.body = self.server.scene.add_frame('/recording/pose', axes_length=0.06, axes_radius=0.0015)
        self.server.scene.add_point_cloud('/recording/start', points=self.positions[:1].astype(np.float32), colors=(255, 255, 255), point_size=0.012)
        self._add_stops()
        self.slider = self.server.gui.add_slider('Elapsed seconds', min=0, max=float(self.times[-1]-self.times[0]), step=0.1, initial_value=0)
        self.play = self.server.gui.add_checkbox('Play recording', initial_value=False)
        self.speed = self.server.gui.add_slider('Playback speed', min=0.5, max=5, step=0.5, initial_value=1)
        self.status = self.server.gui.add_markdown('')
        self.server.gui.add_markdown('**Recorded trajectory, not live.** Teal: complete trajectory. Yellow: last 5 seconds of each stop. Means: red for round 1, blue for round 2. Labels retain original point IDs. Axes and coordinates in metres.\n\nNo registered map was recorded. Optional grey floor samples are a near-static raw-scan reconstruction, not a saved mapping result. No fitted rectangle is substituted.')
        annotation_path = directory / 'operator_annotation.json'
        if annotation_path.exists():
            annotation = json.loads(annotation_path.read_text())
            self.server.gui.add_markdown(annotation['display_note'])
        self.server.gui.add_markdown('Labels are shown exactly as recorded. Physical placement, orientation, lever arm and reference uncertainty require independent verification. Stop jitter is not absolute position accuracy.')
        lines = ['| Round | Stop | Last-5s radial P95 (mm) |', '|---|---|---:|']
        for stop in self.metrics['stops']:
            if 'radial_p95_m' in stop:
                lines.append('| {} | {} | {:.1f} |'.format(stop.get('round_index') or '-', stop['point_id'], 1000*stop['radial_p95_m']))
        lines += ['', '| Segment | Estimated distance (cm) |', '|---|---:|']
        for edge in self.metrics['consecutive_stop_distances']:
            lines.append('| {} → {} | {:.2f} |'.format(edge['from_point'], edge['to_point'], 100*edge['distance_3d_m']))
        self.server.gui.add_markdown('\n'.join(lines))
        self.server.on_client_connect(self._camera)

    def _add_overlay(self, name, positions, color):
        segments = np.stack((positions[:-1], positions[1:]), axis=1).astype(np.float32)
        handle = self.server.scene.add_line_segments('/recording/additional/'+name,
            points=segments, colors=color, line_width=2)
        toggle = self.server.gui.add_checkbox('Show '+name, initial_value=True)

        @toggle.on_update
        def update_visibility(event):
            handle.visible = toggle.value

    def _add_stops(self):
        for index, stop in enumerate(self.metrics['stops']):
            if 'mean_position_m' not in stop:
                continue
            mean = np.asarray(stop['mean_position_m'], dtype=np.float32)
            round_index = stop.get('round_index')
            label = 'R{} {}'.format(round_index, stop['point_id']) if round_index else stop['point_id']
            color = (80, 150, 255) if round_index == 2 else (255, 80, 80)
            selected = self.positions[(self.times >= stop['end_unix_s']-5) & (self.times <= stop['end_unix_s'])]
            self.server.scene.add_point_cloud('/recording/stops/{}/samples'.format(index), points=selected.astype(np.float32), colors=(255, 210, 60), point_size=0.004)
            self.server.scene.add_point_cloud('/recording/stops/{}/mean'.format(index), points=mean[None], colors=color, point_size=0.014)
            self.server.scene.add_label('/recording/stops/{}/label'.format(index), text=label, position=tuple(mean + (0, 0, 0.055 if round_index == 2 else 0.025)))

    def _display_positions(self, positions):
        if self.align_view.value and self.alignment is not None:
            return positions @ np.array(self.alignment['rotation']).T + self.alignment['translation_m']
        return positions

    def _camera(self, client):
        positions = self._display_positions(self.positions)
        center = (positions.min(axis=0)+positions.max(axis=0))/2
        extent = max(float(np.ptp(positions, axis=0).max()), 0.2)
        client.camera.look_at = tuple(center)
        client.camera.position = tuple(center + np.array((0.6, -0.8, 1.5))*extent)
        client.camera.up_direction = (0, 0, 1)

    def run(self):
        """Update only the small cursor during playback; scene geometry is static."""
        previous_time = time.monotonic()
        last_index = None
        last_alignment = self.align_view.value
        try:
            while True:
                now = time.monotonic()
                if self.floor_handle is not None:
                    self.floor_handle.visible = self.show_floor.value
                if self.align_view.value != last_alignment:
                    enabled = self.align_view.value
                    self.scene_frame.wxyz = tuple(self.alignment['rotation_wxyz']) if enabled else (1, 0, 0, 0)
                    self.scene_frame.position = tuple(self.alignment['translation_m']) if enabled else (0, 0, 0)
                    for client in self.server.get_clients().values():
                        self._camera(client)
                    last_alignment, last_index = enabled, None
                if self.play.value:
                    self.slider.value = min(self.slider.max, self.slider.value+(now-previous_time)*self.speed.value)
                    if self.slider.value >= self.slider.max:
                        self.play.value = False
                index = min(int(np.searchsorted(self.times, self.times[0]+self.slider.value)), len(self.times)-1)
                if index != last_index:
                    self.body.position = self.positions[index]
                    self.body.wxyz = self.rotations[index]
                    displayed = self._display_positions(self.positions[index])
                    self.status.content = 'Recorded {:.1f} / {:.1f} s\n\nPosition (m): {:.3f}, {:.3f}, {:.3f}\n\nFrame: {}'.format(self.slider.value, self.slider.max, *displayed, 'floor diagnostic' if self.align_view.value else 'raw camera_init')
                    last_index = index
                previous_time = now
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            self.server.stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('analysis_directory', type=Path)
    parser.add_argument('--port', type=int, default=8080)
    arguments = parser.parse_args()
    RecordingViewer(arguments.analysis_directory, arguments.port).run()


if __name__ == '__main__':
    main()
