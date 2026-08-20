#!/usr/bin/env python3
"""
Republishes nvblox's ESDF as a recordable PointCloud2 on /esdf_recorder/pointcloud.

Supersedes esdf_keepalive.py, which was built on a wrong diagnosis (assumed
nvblox's /nvblox_node/static_esdf_pointcloud was lazy-publish-gated on
subscriber count, fixable with a guaranteed subscriber + matching QoS).
Traced through nvblox_ros source instead of guessing further:
  - nvblox.launch.py forces esdf_mode: "3d" (Fast-Planner needs full 3D
    ESDF, not a 2D slice).
  - nvblox_node.cpp:783 gates the ENTIRE subscriber-driven publish path
    for static_esdf_pointcloud_publisher_ (the get_subscription_count()
    check included) behind `if (params_.esdf_mode == EsdfMode::k2D)`.
  - static_esdf_pointcloud_publisher_->publish() has exactly one call site
    in the whole nvblox_ros source (nvblox_node.cpp:787, inside that same
    k2D-only block). In 3D mode this is dead code -- no subscriber, no QoS,
    no wait time will ever make it publish.
  - The other trigger (visualize_esdf=true on the ESDF service, which sets
    publish_layers_requested_ -> publishLayers()) does NOT publish ESDF at
    all -- LayerPublisher only exposes mesh/tsdf/color/freespace/dynamic-
    occupancy layers (layer_publishing.cpp:596-632), no ESDF layer.
  - The ONLY live path to ESDF data in 3D mode is the
    /nvblox_node/get_esdf_and_gradient service response itself (the same
    one Fast-Planner's sdf_map.cpp::fetchEsdfBlock/onEsdfResponse and
    health_check.py already use). This node just calls that periodically
    and turns the response into a normal recordable topic.

Voxel indexing/sentinel convention matches sdf_map.cpp::onEsdfResponse
exactly: data[x*ny*nz + y*nz + z], origin_m = AABB min corner, voxel
centers at origin + voxel_size*(i+0.5), unobserved sentinel <= -999.0f.
"""
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from nvblox_msgs.srv import EsdfAndGradients

# Centered at origin, generous enough to cover the vio_test.sdf obstacle
# area used in Phase B testing (drone not flying -- fixed AABB is fine
# here; a real in-flight recorder would track vehicle position the way
# sdf_map.cpp's fetchEsdfBlock does, but that's out of scope for this
# record-path verification).
AABB_MIN = (-4.0, -4.0, -1.0)
AABB_SIZE = (8.0, 8.0, 4.0)
FETCH_PERIOD_S = 1.0
UNOBSERVED_SENTINEL = -999.0


class EsdfRecorder(Node):
    def __init__(self):
        super().__init__("esdf_recorder")
        self.cli = self.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")
        self.pub = self.create_publisher(PointCloud2, "/esdf_recorder/pointcloud", 1)
        self.in_flight = False
        self.calls = 0
        self.publishes = 0
        self.timer = self.create_timer(FETCH_PERIOD_S, self.tick)

    def tick(self) -> None:
        if self.in_flight:
            return
        if not self.cli.service_is_ready():
            return
        req = EsdfAndGradients.Request()
        req.update_esdf = True
        req.visualize_esdf = False
        req.use_aabb = True
        req.frame_id = "map"
        req.aabb_min_m = Point(x=AABB_MIN[0], y=AABB_MIN[1], z=AABB_MIN[2])
        req.aabb_size_m = Vector3(x=AABB_SIZE[0], y=AABB_SIZE[1], z=AABB_SIZE[2])
        self.in_flight = True
        self.calls += 1
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_response)

    def on_response(self, future) -> None:
        self.in_flight = False
        resp = future.result()
        if resp is None or not resp.success:
            return

        voxel_size = resp.voxel_size_m
        if voxel_size <= 0.0:
            return
        dims = resp.esdf_and_gradients.layout.dim
        if len(dims) != 3:
            return
        nx, ny, nz = dims[0].size, dims[1].size, dims[2].size
        data = resp.esdf_and_gradients.data
        if nx <= 0 or ny <= 0 or nz <= 0 or len(data) != nx * ny * nz:
            return

        ox, oy, oz = resp.origin_m.x, resp.origin_m.y, resp.origin_m.z

        points = bytearray()
        n_points = 0
        for ix in range(nx):
            for iy in range(ny):
                base = ix * ny * nz + iy * nz
                for iz in range(nz):
                    d = data[base + iz]
                    if d <= UNOBSERVED_SENTINEL:
                        continue
                    x = ox + voxel_size * (ix + 0.5)
                    y = oy + voxel_size * (iy + 0.5)
                    z = oz + voxel_size * (iz + 0.5)
                    points.extend(struct.pack("ffff", x, y, z, d))
                    n_points += 1

        if n_points == 0:
            return

        msg = PointCloud2()
        msg.header = Header(frame_id="map", stamp=self.get_clock().now().to_msg())
        msg.height = 1
        msg.width = n_points
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n_points
        msg.is_dense = True
        msg.data = bytes(points)
        self.pub.publish(msg)
        self.publishes += 1
        if self.publishes == 1 or self.publishes % 10 == 0:
            self.get_logger().info(
                f"esdf_recorder: publish #{self.publishes} ({n_points} voxels, "
                f"{self.calls} service calls so far)"
            )


def main() -> None:
    rclpy.init()
    node = EsdfRecorder()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
