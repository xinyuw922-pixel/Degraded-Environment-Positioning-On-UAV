"""Play aligned odometry tracks against one sensor clock in Viser."""

import argparse
import json
import time
from pathlib import Path

import numpy as np
import viser


def add_stop_markers(server, metrics_file, origin, visible):
    stops = json.loads(metrics_file.read_text())['stops']
    server.gui.add_markdown('停靠点取 FAST-LIO2 各次停稳末 5 秒的平均估计位置。点号沿用原始记录，第一轮第三排顺序待确认。')
    for round_index, color, offset in [(1, (255, 80, 80), 0.035), (2, (75, 140, 255), -0.035)]:
        path = f'/stops/R{round_index}'
        group = server.scene.add_frame(path, show_axes=False, visible=visible)
        toggle = server.gui.add_checkbox(f'FAST-LIO2 第 {round_index} 轮估计', initial_value=visible)
        for stop in stops:
            if stop['round_index'] != round_index:
                continue
            position = np.asarray(stop['mean_position_m']) - origin
            point_id = stop['point_id']
            marker_path = f'{path}/{point_id}'
            server.scene.add_icosphere(marker_path, radius=0.009, color=color, position=position)
            server.scene.add_label(marker_path + '/label', text=f'R{round_index} {point_id}',
                                   position=(0, 0, offset))

        @toggle.on_update
        def toggle_round(event, target=group):
            target.visible = event.target.value


def serve(manifest_path, port):
    manifest = json.loads(manifest_path.read_text())
    tracks = []
    for entry in manifest['tracks']:
        with np.load(entry['file']) as trajectory:
            tracks.append((entry, trajectory['source_stamp_s'], trajectory['positions_m']))
    grid = manifest.get('nominal_grid')
    origin = np.zeros(3)
    if grid:
        stops = json.loads(Path(manifest['stop_metrics']).read_text())['stops']
        anchor = next(s for s in stops if s['round_index'] == 1 and s['point_id'] == 'P01')
        origin = np.asarray(anchor['mean_position_m'])
        tracks = [(entry, stamps, positions - origin) for entry, stamps, positions in tracks]
    start = min(stamps[0] for _, stamps, _ in tracks)
    duration = max(stamps[-1] for _, stamps, _ in tracks) - start
    server = viser.ViserServer(host='127.0.0.1', port=port, label='室内定位轨迹对比')
    server.scene.set_up_direction('+z')
    server.scene.add_grid('/grid', width=3, height=3, cell_size=0.1, plane='xy')
    server.gui.add_markdown(manifest['note'])
    if 'stop_metrics' in manifest:
        add_stop_markers(server, Path(manifest['stop_metrics']), origin, visible=not grid)
    if grid:
        server.gui.add_markdown('白点为标称网格。共同显示原点取第一轮 FAST-LIO2 P01；列向 −Y、行向 −X，地面方向为近似。')
        for row in range(grid['rows']):
            for col in range(grid['columns']):
                label = f"P{row * grid['columns'] + col + 1:02d}"
                position = (-row * grid['spacing_m'], -col * grid['spacing_m'], 0.)
                path = '/nominal/' + label
                server.scene.add_icosphere(path, radius=0.009, color=(255, 255, 255), position=position)
                server.scene.add_label(path + '/label', text=label, position=(0, 0, 0.025))
    slider = server.gui.add_slider('传感器时间（秒）', min=0., max=float(duration), step=0.05, initial_value=0.)
    play = server.gui.add_checkbox('播放', initial_value=False)
    speed = server.gui.add_slider('播放倍速', min=1, max=20, step=1, initial_value=5)
    controls = []
    for entry, stamps, positions in tracks:
        name, color = entry['name'], tuple(entry['color'])
        path = '/tracks/' + name
        line = server.scene.add_line_segments(path + '/line',
            points=np.stack([positions[:-1], positions[1:]], axis=1).astype(np.float32),
            colors=color, line_width=2)
        marker = server.scene.add_icosphere(path + '/cursor', radius=0.012, color=color)
        toggle = server.gui.add_checkbox(name, initial_value=True)
        controls.append((toggle, line, marker, stamps, positions))

    @server.on_client_connect
    def set_camera(client):
        positions = tracks[0][2]
        center = (positions.min(0) + positions.max(0)) / 2
        client.camera.position = center + np.array([0.65, -0.9, 1.5]) * max(np.ptp(positions, axis=0))
        client.camera.look_at = center
        client.camera.up_direction = (0, 0, 1)

    previous = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if play.value:
                slider.value = min(duration, slider.value + (now - previous) * speed.value)
                if slider.value >= duration:
                    play.value = False
            previous = now
            stamp = start + slider.value
            for toggle, line, marker, stamps, positions in controls:
                if line.visible != toggle.value:
                    line.visible = toggle.value
                visible = bool(toggle.value and stamps[0] <= stamp <= stamps[-1])
                if marker.visible != visible:
                    marker.visible = visible
                if visible:
                    index = np.searchsorted(stamps, stamp, side='right') - 1
                    marker.position = positions[index]
            time.sleep(0.1)
    finally:
        server.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    serve(args.manifest, args.port)
