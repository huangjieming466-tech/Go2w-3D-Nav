#!/usr/bin/env python3
"""Experimental timestamp-aware fusion of MID-360 and Unitree UTLIDAR clouds.

Both sensor extrinsics are expressed relative to base_link. Fused points are
converted back to the MID-360 frame so the existing Super-LIO sensor
extrinsic remains unchanged.
"""

import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField


_FIELD_FORMATS = {
    PointField.FLOAT32: 'f4',
    PointField.FLOAT64: 'f8',
    PointField.UINT8: 'u1',
    PointField.UINT16: 'u2',
    PointField.UINT32: 'u4',
    PointField.INT8: 'i1',
    PointField.INT16: 'i2',
    PointField.INT32: 'i4',
}
_POINT_DTYPE = np.dtype([
    ('time_ns', '<i8'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
    ('intensity', '<f4'), ('tag', 'u1'),
])
_OUTPUT_DTYPE = np.dtype({
    'names': ('x', 'y', 'z', 'intensity', 'tag', 'timestamp'),
    'formats': ('<f4', '<f4', '<f4', '<f4', 'u1', '<f8'),
    'offsets': (0, 4, 8, 12, 16, 24),
    'itemsize': 32,
})


@dataclass
class TimedCloud:
    points: np.ndarray
    frame_id: str

    @property
    def start_ns(self):
        return int(self.points['time_ns'][0])

    @property
    def end_ns(self):
        return int(self.points['time_ns'][-1])

    @property
    def span_ns(self):
        return self.end_ns - self.start_ns


class ArrivalClockOffsetEstimator:
    """Estimate UTLIDAR-to-MID timestamp offset from near-simultaneous arrivals."""

    def __init__(self, arrival_skew_ns, window_size, min_samples, fallback_offset_ns):
        self.arrival_skew_ns = int(arrival_skew_ns)
        self.mid_arrivals = deque()
        self.utli_arrivals = deque()
        self.offset_samples = deque(maxlen=max(3, int(window_size)))
        self.min_samples = max(3, int(min_samples))
        self.offset_ns = int(fallback_offset_ns)
        self.locked = False

    def add_mid(self, arrival_ns, scan_end_ns):
        self.mid_arrivals.append((int(arrival_ns), int(scan_end_ns)))
        return self._match()

    def add_utlidar(self, arrival_ns, scan_end_ns):
        self.utli_arrivals.append((int(arrival_ns), int(scan_end_ns)))
        return self._match()

    def _match(self):
        updated = False
        while self.mid_arrivals and self.utli_arrivals:
            mid_arrival, mid_end = self.mid_arrivals[0]
            utli_arrival, utli_end = self.utli_arrivals[0]
            arrival_delta = mid_arrival - utli_arrival
            if abs(arrival_delta) <= self.arrival_skew_ns:
                self.mid_arrivals.popleft()
                self.utli_arrivals.popleft()
                self.offset_samples.append(mid_end - utli_end)
                if len(self.offset_samples) >= self.min_samples:
                    self.offset_ns = int(np.median(self.offset_samples))
                    self.locked = True
                    updated = True
            elif arrival_delta < 0:
                self.mid_arrivals.popleft()
            else:
                self.utli_arrivals.popleft()

        while len(self.mid_arrivals) > 64:
            self.mid_arrivals.popleft()
        while len(self.utli_arrivals) > 64:
            self.utli_arrivals.popleft()
        return updated


def _field_map(msg):
    return {field.name: field for field in msg.fields}


def _cloud_view(msg, fields):
    names = list(fields)
    formats = []
    offsets = []
    endian = '>' if msg.is_bigendian else '<'
    for name in names:
        field = fields[name]
        if field.count != 1 or field.datatype not in _FIELD_FORMATS:
            raise ValueError(f'unsupported PointCloud2 field layout for {name!r}')
        code = _FIELD_FORMATS[field.datatype]
        formats.append(code if code in ('u1', 'i1') else endian + code)
        offsets.append(field.offset)
    dtype = np.dtype({
        'names': names, 'formats': formats, 'offsets': offsets,
        'itemsize': msg.point_step,
    })
    count = msg.width * msg.height
    if count == 0 or len(msg.data) < count * msg.point_step:
        raise ValueError('empty or truncated PointCloud2')
    return np.frombuffer(msg.data, dtype=dtype, count=count)


def _stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


class Mid360UtlidarFusion(Node):
    def __init__(self):
        super().__init__('mid360_utlidar_fusion')
        self.declare_parameter('mid360_topic', '/livox/lidar')
        self.declare_parameter('utlidar_topic', '/utlidar/cloud')
        self.declare_parameter('output_topic', '/lio/mid360_utlidar_fused')
        self.declare_parameter('output_frame', '')
        self.declare_parameter('max_combined_scan_duration', 0.22)
        self.declare_parameter('max_pair_end_skew_sec', 0.10)
        self.declare_parameter('max_pending_scans', 8)
        self.declare_parameter('mid360_decimation', 3)
        self.declare_parameter('utlidar_blind_range', 0.5)
        self.declare_parameter('utlidar_clock_offset_sec', 402.228)
        self.declare_parameter('auto_estimate_utlidar_clock_offset', True)
        self.declare_parameter('clock_calibration_arrival_skew_sec', 0.10)
        self.declare_parameter('clock_calibration_window_size', 31)
        self.declare_parameter('clock_calibration_min_samples', 7)
        self.declare_parameter('utlidar_aligned_debug_topic', '/lio/utlidar_aligned_debug')
        # T_base_mid360 comes from T_base_imu * T_imu_mid360. The Go2 URDF
        # has aligned base_link/IMU axes; T_imu_mid360 comes from Super-LIO.
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('base_t_mid360', [0.16143, 0.0, 0.12262])
        self.declare_parameter('base_rpy_mid360', [0.0, math.radians(13.0), 0.0])
        self.declare_parameter('base_t_utlidar', [0.28216, 0.0, 0.0])
        self.declare_parameter(
            'base_rpy_utlidar', [-2.92072, -0.141324, -1.01053]
        )
        self.base_t_mid = np.asarray(
            self.get_parameter('base_t_mid360').value, dtype=np.float64
        )
        self.base_r_mid = rpy_matrix(*map(
            float, self.get_parameter('base_rpy_mid360').value
        ))
        self.base_t_utli = np.asarray(
            self.get_parameter('base_t_utlidar').value, dtype=np.float64
        )
        self.base_r_utli = rpy_matrix(*map(
            float, self.get_parameter('base_rpy_utlidar').value
        ))
        if self.base_t_mid.shape != (3,) or self.base_t_utli.shape != (3,):
            raise ValueError('base-to-sensor translations must have 3 values')
        if str(self.get_parameter('base_frame').value) != 'base_link':
            raise ValueError('fusion extrinsics are defined in base_link')
        self.max_span_ns = int(
            float(self.get_parameter('max_combined_scan_duration').value) * 1e9
        )
        self.utlidar_clock_offset_ns = int(
            float(self.get_parameter('utlidar_clock_offset_sec').value) * 1e9
        )
        self.auto_estimate_clock_offset = bool(
            self.get_parameter('auto_estimate_utlidar_clock_offset').value
        )
        self.clock_offset_estimator = ArrivalClockOffsetEstimator(
            float(self.get_parameter('clock_calibration_arrival_skew_sec').value) * 1e9,
            self.get_parameter('clock_calibration_window_size').value,
            self.get_parameter('clock_calibration_min_samples').value,
            self.utlidar_clock_offset_ns,
        )
        self.clock_offset_locked = not self.auto_estimate_clock_offset
        self.mid_frame_id = ''
        self.utlidar_aligned_debug_topic = str(
            self.get_parameter('utlidar_aligned_debug_topic').value
        )
        self.output_frame = str(self.get_parameter('output_frame').value)
        self.mid_queue = []
        self.utli_queue = []
        self.max_pair_skew_ns = int(
            float(self.get_parameter('max_pair_end_skew_sec').value) * 1e9
        )
        self.max_pending_scans = max(2, int(self.get_parameter('max_pending_scans').value))
        self.mid360_decimation = max(1, int(self.get_parameter('mid360_decimation').value))
        self.utlidar_blind_range = max(
            0.0, float(self.get_parameter('utlidar_blind_range').value)
        )
        self.last_output_start_ns = 0
        self.stats = {
            'mid_received': 0,
            'utlidar_received': 0,
            'parse_errors': 0,
            'pairs': 0,
            'span_rejections': 0,
            'out_of_order_rejections': 0,
            'published_fused': 0,
            'published_mid_only': 0,
            'published_utlidar_debug': 0,
            'mid_callback_max_sec': 0.0,
            'utlidar_callback_max_sec': 0.0,
            'mid_parse_max_sec': 0.0,
            'utlidar_parse_max_sec': 0.0,
            'match_max_sec': 0.0,
            'publish_max_sec': 0.0,
        }

        input_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            PointCloud2, str(self.get_parameter('output_topic').value), output_qos
        )
        self.utlidar_debug_publisher = self.create_publisher(
            PointCloud2, self.utlidar_aligned_debug_topic, output_qos
        )
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('mid360_topic').value),
            self._mid_callback,
            input_qos,
        )
        self.create_timer(5.0, self._report_stats)
        self.create_subscription(
            PointCloud2,
            str(self.get_parameter('utlidar_topic').value),
            self._utli_callback,
            input_qos,
        )
        offset_mode = (
            'online soft-clock alignment' if self.auto_estimate_clock_offset
            else 'fixed fallback'
        )
        self.get_logger().warn(
            'Experimental fusion enabled. UTLIDAR extrinsic is a URDF-derived initial '
            'estimate; verify cloud alignment and timing before robot motion. '
            f'Clock mode={offset_mode}; fallback offset='
            f'{self.utlidar_clock_offset_ns * 1e-9:.6f}s.'
        )

    def _update_clock_estimate(self, updated):
        if not self.auto_estimate_clock_offset:
            return
        if not self.clock_offset_estimator.locked:
            return
        self.utlidar_clock_offset_ns = self.clock_offset_estimator.offset_ns
        if not self.clock_offset_locked:
            self.clock_offset_locked = True
            self.get_logger().warn(
                'UTLIDAR soft-clock alignment locked: '
                f'offset={self.utlidar_clock_offset_ns * 1e-9:.6f}s from '
                f'{len(self.clock_offset_estimator.offset_samples)} arrival-matched scans.'
            )
        elif updated:
            self.get_logger().info(
                'UTLIDAR soft-clock offset updated: '
                f'{self.utlidar_clock_offset_ns * 1e-9:.6f}s.',
                throttle_duration_sec=5.0,
            )

    def _mid_callback(self, msg):
        started = time.perf_counter()
        arrival_ns = time.monotonic_ns()
        self.stats['mid_received'] += 1
        try:
            parse_started = time.perf_counter()
            mid = self._parse_mid360(msg)
            self.mid_frame_id = mid.frame_id
            if self.auto_estimate_clock_offset:
                updated = self.clock_offset_estimator.add_mid(arrival_ns, mid.end_ns)
                self._update_clock_estimate(updated)
            self.stats['mid_parse_max_sec'] = max(
                self.stats['mid_parse_max_sec'], time.perf_counter() - parse_started
            )
            if not self.clock_offset_locked:
                self._publish_points(mid.points, mid.frame_id)
                return
            self.mid_queue.append(mid)
            self.mid_queue.sort(key=lambda cloud: cloud.end_ns)
            self._match_scans()
        except (KeyError, ValueError, TypeError) as exc:
            self.stats['parse_errors'] += 1
            self.get_logger().error(f'Rejecting MID-360 cloud: {exc}')
        finally:
            self.stats['mid_callback_max_sec'] = max(
                self.stats['mid_callback_max_sec'], time.perf_counter() - started
            )

    def _utli_callback(self, msg):
        started = time.perf_counter()
        arrival_ns = time.monotonic_ns()
        self.stats['utlidar_received'] += 1
        try:
            parse_started = time.perf_counter()
            raw_utli = self._parse_utlidar(msg)
            if self.auto_estimate_clock_offset:
                updated = self.clock_offset_estimator.add_utlidar(arrival_ns, raw_utli.end_ns)
                self._update_clock_estimate(updated)
            self.stats['utlidar_parse_max_sec'] = max(
                self.stats['utlidar_parse_max_sec'], time.perf_counter() - parse_started
            )
            if not self.clock_offset_locked:
                return
            adjusted_points = raw_utli.points.copy()
            adjusted_points['time_ns'] += self.utlidar_clock_offset_ns
            utli = TimedCloud(adjusted_points, raw_utli.frame_id)
            self._publish_utlidar_debug(utli)
            self.utli_queue.append(utli)
            self.utli_queue.sort(key=lambda cloud: cloud.end_ns)
            self._match_scans()
        except (KeyError, ValueError, TypeError) as exc:
            self.stats['parse_errors'] += 1
            self.get_logger().error(f'Rejecting UTLIDAR cloud: {exc}')
        finally:
            self.stats['utlidar_callback_max_sec'] = max(
                self.stats['utlidar_callback_max_sec'], time.perf_counter() - started
            )

    def _report_stats(self):
        self.get_logger().info(
            'Fusion totals: '
            + ', '.join(f'{name}={value}' for name, value in self.stats.items())
            + f', clock_offset={self.utlidar_clock_offset_ns * 1e-9:.6f}s'
            + f', clock_locked={self.clock_offset_locked}'
            + f', clock_samples={len(self.clock_offset_estimator.offset_samples)}'
            + f', queues=({len(self.mid_queue)},{len(self.utli_queue)}).'
        )

    def _validate_xyz(self, fields):
        for name in ('x', 'y', 'z'):
            if name not in fields or fields[name].datatype != PointField.FLOAT32:
                raise ValueError(f'required float32 field {name!r} missing')

    def _parse_mid360(self, msg):
        fields = _field_map(msg)
        self._validate_xyz(fields)
        intensity_name = 'reflectivity' if 'reflectivity' in fields else 'intensity'
        for name in (intensity_name, 'tag', 'timestamp'):
            if name not in fields:
                raise ValueError(f'required Livox field {name!r} missing')
        if fields['timestamp'].datatype != PointField.FLOAT64:
            raise ValueError('Livox timestamp must be float64 nanoseconds')
        raw = _cloud_view(msg, {
            'x': fields['x'], 'y': fields['y'], 'z': fields['z'],
            'intensity': fields[intensity_name], 'tag': fields['tag'],
            'timestamp': fields['timestamp'],
        })
        times = raw['timestamp'].astype(np.float64)
        header_ns = _stamp_ns(msg.header.stamp)
        if np.any(np.abs(times - header_ns) > 2_000_000_000):
            raise ValueError('Livox timestamp is not absolute ROS-epoch nanoseconds')
        mask = np.isfinite(times)
        for name in ('x', 'y', 'z', 'intensity'):
            mask &= np.isfinite(raw[name])
        if not np.any(mask):
            raise ValueError('no finite MID-360 points')
        # Match the original Super-LIO filter_rate=3 before serialization.
        # The fused launch sets LIO's filter_rate to 1 to avoid double thinning.
        selected = np.flatnonzero(mask)[::self.mid360_decimation]
        points = np.empty(len(selected), dtype=_POINT_DTYPE)
        points['time_ns'] = times[selected].astype(np.int64)
        for name in ('x', 'y', 'z', 'intensity'):
            points[name] = raw[name][selected]
        points['tag'] = raw['tag'][selected].astype(np.uint8)
        points.sort(order='time_ns')
        return TimedCloud(points, self.mid_frame_id or 'livox_frame')

    def _parse_utlidar(self, msg):
        fields = _field_map(msg)
        self._validate_xyz(fields)
        if 'intensity' not in fields or 'time' not in fields:
            raise ValueError('UTLIDAR requires intensity and per-point time fields')
        if fields['time'].datatype not in (PointField.FLOAT32, PointField.FLOAT64):
            raise ValueError('UTLIDAR time must be floating-point seconds')
        raw = _cloud_view(msg, {
            'x': fields['x'], 'y': fields['y'], 'z': fields['z'],
            'intensity': fields['intensity'], 'time': fields['time'],
        })
        header_ns = _stamp_ns(msg.header.stamp)
        relative = raw['time'].astype(np.float64)
        if np.any((relative < -1e-4) | (relative > 0.25)):
            raise ValueError('UTLIDAR per-point time is outside the expected scan window')
        xyz = np.column_stack((raw['x'], raw['y'], raw['z'])).astype(np.float64)
        intensities = raw['intensity'].astype(np.float32)
        mask = np.isfinite(relative) & np.isfinite(intensities) & np.isfinite(xyz).all(axis=1)
        mask &= np.einsum('ij,ij->i', xyz, xyz) >= self.utlidar_blind_range ** 2
        if not np.any(mask):
            raise ValueError('no finite UTLIDAR points')
        # Convert UTLIDAR -> base_link -> MID-360 frame. This keeps the
        # published fused cloud compatible with the unchanged LIO extrinsic.
        xyz_base = xyz[mask] @ self.base_r_utli.T + self.base_t_utli
        xyz = (xyz_base - self.base_t_mid) @ self.base_r_mid
        points = np.empty(int(np.count_nonzero(mask)), dtype=_POINT_DTYPE)
        points['time_ns'] = (
            header_ns + self.utlidar_clock_offset_ns
            + np.rint(relative[mask] * 1e9).astype(np.int64)
        )
        points['x'], points['y'], points['z'] = xyz.T.astype(np.float32)
        points['intensity'] = intensities[mask]
        points['tag'] = 0x10
        points.sort(order='time_ns')
        # Coordinates have been transformed into the MID-360 frame; keep the
        # debug cloud's frame consistent so RViz does not apply the UTLIDAR TF again.
        return TimedCloud(points, self.mid_frame_id or msg.header.frame_id)

    def _match_scans(self):
        started = time.perf_counter()
        self._match_scans_impl()
        self.stats['match_max_sec'] = max(
            self.stats['match_max_sec'], time.perf_counter() - started
        )

    def _match_scans_impl(self):
        while self.mid_queue and self.utli_queue:
            mid = self.mid_queue[0]
            mid_end = mid.end_ns
            latest_utli_end = self.utli_queue[-1].end_ns
            if latest_utli_end < mid_end:
                if mid_end - latest_utli_end > self.max_pair_skew_ns:
                    self.utli_queue = [
                        cloud for cloud in self.utli_queue
                        if cloud.end_ns >= mid_end - self.max_pair_skew_ns
                    ]
                    self.mid_queue.pop(0)
                    self._publish_points(mid.points, mid.frame_id)
                    continue
                return

            match_idx = min(
                range(len(self.utli_queue)),
                key=lambda i: abs(self.utli_queue[i].end_ns - mid_end),
            )
            utli = self.utli_queue[match_idx]
            end_delta_ns = mid_end - utli.end_ns
            if abs(end_delta_ns) > self.max_pair_skew_ns:
                # The newest UTLIDAR sweep is already later than this MID sweep,
                # so no future UTLIDAR frame can be a better match.
                self.mid_queue.pop(0)
                self._publish_points(mid.points, mid.frame_id)
                continue

            self.mid_queue.pop(0)
            self.utli_queue.pop(match_idx)
            self._publish_pair(mid, utli)

            # Frames older than the consumed pair cannot match a future MID scan.
            cutoff = min(mid_end, utli.end_ns) - self.max_pair_skew_ns
            self.utli_queue = [
                cloud for cloud in self.utli_queue if cloud.end_ns >= cutoff
            ]

        if len(self.mid_queue) > self.max_pending_scans:
            del self.mid_queue[:-self.max_pending_scans]
        if len(self.utli_queue) > self.max_pending_scans:
            del self.utli_queue[:-self.max_pending_scans]

    def _publish_pair(self, mid, utli):
        self.stats['pairs'] += 1
        start_ns = min(mid.start_ns, utli.start_ns)
        end_ns = max(mid.end_ns, utli.end_ns)
        if end_ns - start_ns > self.max_span_ns:
            self.stats['span_rejections'] += 1
            self.get_logger().warn(
                'UTLIDAR sweep rejected; forwarding MID-360 only because combined '
                'time span exceeds '
                f'{self.max_span_ns * 1e-9:.3f} s '
                f'(MID={mid.span_ns * 1e-9:.3f}s, '
                f'UTLIDAR={utli.span_ns * 1e-9:.3f}s, '
                f'end_delta={(mid.end_ns - utli.end_ns) * 1e-9:.3f}s).',
                throttle_duration_sec=2.0,
            )
            self._publish_points(mid.points, mid.frame_id)
            return
        if start_ns <= self.last_output_start_ns:
            self.stats['out_of_order_rejections'] += 1
            self.get_logger().warn(
                'UTLIDAR sweep rejected; forwarding MID-360 only due to out-of-order pair.',
                throttle_duration_sec=2.0,
            )
            self._publish_points(mid.points, mid.frame_id)
            return

        points = np.concatenate((mid.points, utli.points))
        points.sort(order='time_ns')
        self._publish_points(points, self.output_frame or mid.frame_id, fused=True)
        self.get_logger().info(
            f'Published fused sweep: MID-360={len(mid.points)} UTLIDAR={len(utli.points)} '
            f'span={(end_ns - start_ns) * 1e-9:.3f}s',
            throttle_duration_sec=5.0,
        )

    def _make_pointcloud(self, points, frame_id):
        start_ns = int(points['time_ns'][0])
        packed = np.empty(len(points), dtype=_OUTPUT_DTYPE)
        for name in ('x', 'y', 'z', 'intensity', 'tag'):
            packed[name] = points[name]
        packed['timestamp'] = points['time_ns'].astype(np.float64)
        out = PointCloud2()
        out.header.stamp.sec, out.header.stamp.nanosec = divmod(start_ns, 1_000_000_000)
        out.header.frame_id = frame_id or 'livox_frame'
        out.height = 1
        out.width = len(points)
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='tag', offset=16, datatype=PointField.UINT8, count=1),
            PointField(name='timestamp', offset=24, datatype=PointField.FLOAT64, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 32
        out.row_step = out.point_step * out.width
        out.is_dense = False
        out.data = packed.tobytes()
        return out

    def _publish_utlidar_debug(self, cloud):
        if self.utlidar_debug_publisher.get_subscription_count() == 0:
            return
        self.utlidar_debug_publisher.publish(
            self._make_pointcloud(cloud.points, cloud.frame_id)
        )
        self.stats['published_utlidar_debug'] += 1

    def _publish_points(self, points, frame_id, fused=False):
        started = time.perf_counter()
        start_ns = int(points['time_ns'][0])
        if start_ns <= self.last_output_start_ns:
            return
        self.publisher.publish(self._make_pointcloud(points, frame_id))
        self.stats['published_fused' if fused else 'published_mid_only'] += 1
        self.last_output_start_ns = start_ns
        self.stats['publish_max_sec'] = max(
            self.stats['publish_max_sec'], time.perf_counter() - started
        )


def main(args=None):
    rclpy.init(args=args)
    node = Mid360UtlidarFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
