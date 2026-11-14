#!/usr/bin/env python3
"""Exercise installed launch/node/TF/TUM output using deterministic synthetic scans.

Run after sourcing install/setup.bash. No datasets or third-party Python packages
are needed beyond ROS 2. This is a smoke test, not real-data validation.
"""
import math
import os
import random
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage


def spin_until(node, predicate, timeout=15.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError("Timed out waiting for ROS output")


def publish_until_received(node, publisher, msg, predicate, timeout=15.0):
    """Best-effort input can lose its first sample during DDS discovery.

    Retransmit the same stamped scan until the reliable output arrives. The node
    drops duplicate stamps, so retries cannot add duplicate trajectory entries.
    """
    end = time.monotonic() + timeout
    next_send = 0.0
    while time.monotonic() < end:
        now = time.monotonic()
        if now >= next_send:
            publisher.publish(msg)
            next_send = now + 0.3
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise AssertionError(f"Timed out receiving pose/TF for scan {msg.header.stamp.sec}")


def main():
    rclpy.init()
    node = rclpy.create_node("odometry_smoke_test")
    received, transforms, paths = {}, {}, []
    key = lambda stamp: (stamp.sec, stamp.nanosec)
    subscriptions = [
        node.create_subscription(Odometry, "/odometry", lambda m: received.__setitem__(key(m.header.stamp), m), 100),
        node.create_subscription(TFMessage, "/tf", lambda m: [transforms.__setitem__(key(t.header.stamp), t) for t in m.transforms], 100),
        node.create_subscription(RosPath, "/trajectory", lambda m: paths.append(m), 10),
    ]
    publisher = node.create_publisher(PointCloud2, "/synthetic/points", qos_profile_sensor_data)
    rng = random.Random(19)
    world = [(rng.uniform(-4, 4), rng.uniform(-4, 4), rng.uniform(-1, 3)) for _ in range(1800)]
    with tempfile.TemporaryDirectory(prefix="lidar-smoke-") as directory:
        trajectory = Path(directory) / "estimate.txt"
        log = Path(directory) / "node.log"
        with log.open("w") as stream:
            process = subprocess.Popen([
                "ros2", "launch", "st3821671", "odom.launch.py",
                "points_topic:=/synthetic/points", "rviz:=false",
                f"trajectory_path:={trajectory}",
            ], stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                spin_until(node, lambda: publisher.get_subscription_count() > 0)
                # Wait for the separate DDS output endpoints too.
                spin_until(node, lambda: all(node.count_publishers(t) > 0 for t in ["/odometry", "/tf", "/trajectory"]))
                for i in range(20):
                    x, y, z, yaw = i * 0.04, i * 0.01, i * 0.01, i * 0.005
                    c, s = math.cos(yaw), math.sin(yaw)
                    points = []
                    for wx, wy, wz in world:
                        dx, dy = wx - x, wy - y
                        points.append((c * dx + s * dy, -s * dx + c * dy, wz - z, 25.0))
                    msg = PointCloud2()
                    msg.header.frame_id = "synthetic_lidar"
                    msg.header.stamp.sec = 100 + i
                    msg.header.stamp.nanosec = 123456789
                    msg.width, msg.height = len(points), 1
                    msg.fields = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                                  for name, offset in [("x", 0), ("y", 4), ("z", 8), ("intensity", 12)]]
                    msg.point_step, msg.row_step = 16, len(points) * 16
                    msg.data = b"".join(struct.pack("<ffff", *p) for p in points)
                    msg.is_bigendian = False
                    stamp = key(msg.header.stamp)
                    publish_until_received(node, publisher, msg,
                                           lambda: stamp in received and stamp in transforms)
                    odom, tf = received[stamp], transforms[stamp]
                    assert odom.header.frame_id == "odom"
                    assert odom.child_frame_id == tf.child_frame_id == "synthetic_lidar"
                    p = odom.pose.pose.position
                    error = math.dist((p.x, p.y, p.z), (x, y, z))
                    assert error < 0.05, f"Pose error at frame {i}: {error}"
                    assert math.dist((tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z),
                                     (p.x, p.y, p.z)) < 1e-9
                    q = odom.pose.pose.orientation
                    estimated_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                    assert abs(estimated_yaw - yaw) < 0.02
                    assert math.isclose(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w, 1.0, abs_tol=1e-6)
                    assert process.poll() is None, "Node exited unexpectedly"
                spin_until(node, lambda: paths and len(paths[-1].poses) == 20)
                rows = trajectory.read_text().splitlines()
                assert len(rows) == 20, f"Expected 20 TUM rows, got {len(rows)}"
                assert rows[-1].split()[0] == "119.123456789", "Timestamp precision lost"
                assert all(len(row.split()) == 8 for row in rows)
                assert all(math.isfinite(float(v)) for row in rows for v in row.split())
                print("PASS ROS launch, topic remapping, 20 poses, TF direction, path and TUM output")
            except BaseException:
                stream.flush()
                print(log.read_text())
                raise
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
    del subscriptions
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
