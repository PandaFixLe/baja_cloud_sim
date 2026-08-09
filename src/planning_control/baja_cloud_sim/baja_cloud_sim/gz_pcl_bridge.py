#!/usr/bin/env python3
"""
Gazebo -> ROS PointCloud2 bridge for LiDAR.

The distro's `ros_gz_bridge` (ros-humble-ros-gzharmonic-bridge) corrupts the
`gz.msgs.PointCloudPacked` -> sensor_msgs/msg/PointCloud2 conversion (every
point comes out as -inf). This node subscribes to the raw Gazebo Transport
topic directly and builds a correct ROS PointCloud2, working around that bug.

It is intended for simulation bring-up only (not used on the real vehicle).
"""
import struct
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField

import gz.transport13 as gz_transport
from gz.transport13 import Node as GzNode
# gz.msgs PointCloudPacked protobuf (Python bindings shipped with gz-msgs)
try:
    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked as GzPointCloudPacked
except Exception:  # pragma: no cover
    import sys
    sys.path.insert(0, '/usr/lib/python3/dist-packages/gz/msgs10')
    from pointcloud_packed_pb2 import PointCloudPacked as GzPointCloudPacked

# ROS PointField datatype constants
PF_INT8    = 1
PF_UINT8   = 2
PF_INT16   = 3
PF_UINT16  = 4
PF_INT32   = 5
PF_UINT32  = 6
PF_FLOAT32 = 7
PF_FLOAT64 = 8

# Gazebo's gpu_lidar has a bug: it labels FLOAT32 fields (x, y, z, intensity)
# as UINT32 (datatype=6) in the PointCloudPacked protobuf, even though the
# actual bytes are float32.  Map known float field names to FLOAT32 so
# downstream consumers (pointcloud_filter, patchwork++, RViz) parse correctly.
_FLOAT_FIELD_NAMES = {'x', 'y', 'z', 'intensity'}


class GzPclBridge(Node):
    def __init__(self):
        super().__init__('gz_pcl_bridge')

        self.declare_parameter('gz_topic', '/lidar/points/points')
        self.declare_parameter('ros_topic', '/lidar/points')
        self.declare_parameter('frame_id', 'baja_vehicle/base_link/lidar')
        gz_topic = self.get_parameter('gz_topic').value
        ros_topic = self.get_parameter('ros_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(
            PointCloud2, ros_topic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        self._gz = GzNode()
        ok = self._gz.subscribe(GzPointCloudPacked, gz_topic, self._on_gz_msg)
        if not ok:
            self.get_logger().error(f'Failed to subscribe Gazebo topic {gz_topic}')
        else:
            self.get_logger().info(f'gz_pcl_bridge: subscribed {gz_topic} -> ROS {ros_topic}')

    def _on_gz_msg(self, msg: GzPointCloudPacked):
        try:
            width = msg.width
            height = msg.height
            if width == 0 or height == 0:
                return
            point_step = msg.point_step
            data = bytes(msg.data)

            # Build ROS fields from the packed field descriptors.
            # Fix Gazebo's datatype bug: float32 fields are mislabeled as
            # UINT32(6).  Correct them to FLOAT32(7) by field name.
            ros_fields = []
            for f in msg.field:
                dtype = f.datatype
                if f.name in _FLOAT_FIELD_NAMES and dtype == PF_UINT32:
                    dtype = PF_FLOAT32
                ros_fields.append(PointField(
                    name=f.name,
                    offset=f.offset,
                    datatype=dtype,
                    count=f.count if f.count > 0 else 1,
                ))

            out = PointCloud2()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            out.height = height
            out.width = width
            out.fields = ros_fields
            out.is_bigendian = False
            out.point_step = point_step
            out.row_step = width * point_step
            out.is_dense = False
            out.data = data
            self.pub.publish(out)
        except Exception as e:  # pragma: no cover
            self.get_logger().warn(f'gz_pcl_bridge parse error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GzPclBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
