"""Decode XYZ fields from ROS PointCloud2 without per-point Python loops."""

import math

import numpy as np


def xyz_points(message, limit):
    """Return at most limit finite XYZ points, respecting row padding and endian."""
    fields = {field.name: field for field in message.fields}
    formats = {7: 'f4', 8: 'f8'}
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype({
        'names': ['x', 'y', 'z'],
        'formats': [endian + formats[fields[name].datatype] for name in ('x', 'y', 'z')],
        'offsets': [fields[name].offset for name in ('x', 'y', 'z')],
        'itemsize': message.point_step,
    })
    structured = np.ndarray(
        (message.height, message.width), dtype=dtype, buffer=message.data,
        strides=(message.row_step, message.point_step),
    ).reshape(-1)
    structured = structured[::max(1, math.ceil(len(structured) / limit))]
    points = np.column_stack([structured[name] for name in ('x', 'y', 'z')]).astype(np.float32)
    return points[np.isfinite(points).all(axis=1)]
