#!/usr/bin/env python3
"""Publish the generated TCP strokes to RViz. Contains no robot command publisher."""
import argparse
import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler


class WritingPreview(Node):
    def __init__(self, data):
        super().__init__('guoqing_writing_preview')
        self.data = data
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.markers = self.create_publisher(MarkerArray,'/rebotarm_writing/strokes',qos)
        self.path = self.create_publisher(RosPath,'/rebotarm_writing/tcp_path',qos)
        self.timer = self.create_timer(1.,self.publish)
        self.publish()

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        array = MarkerArray()
        for i, stroke in enumerate(self.data['strokes']):
            marker = Marker()
            marker.header.frame_id = self.data['frame_id']
            marker.header.stamp = stamp
            marker.ns = 'writing_tcp_strokes'
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.
            marker.scale.x = .0018
            rgb = stroke['color'].lstrip('#')
            marker.color.r,marker.color.g,marker.color.b = [int(rgb[j:j+2],16)/255 for j in (0,2,4)]
            marker.color.a = 1.
            # FK samples of the actual joint interpolation, not a text label.
            marker.points = [Point(x=float(x),y=float(y),z=float(z)) for x,y,z in stroke['actual_points']]
            array.markers.append(marker)
        self.markers.publish(array)
        path = RosPath()
        path.header.frame_id,path.header.stamp = self.data['frame_id'],stamp
        for sample in self.data['samples']:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x,pose.pose.position.y,pose.pose.position.z = sample['pose'][:3]
            # The full Path includes lifted transfers; it is disabled in RViz by default.
            q = quaternion_from_euler(*sample['pose'][3:])
            pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = map(float,q)
            path.poses.append(pose)
        self.path.publish(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path',type=Path,default=Path(__file__).resolve().parents[1]/'examples/writing/guoqing_kuaile.path.json')
    args, ros_args = parser.parse_known_args()
    data = json.loads(args.path.read_text())
    if data.get('frame_id') != 'base_link' or len(data.get('strokes',[])) != 26:
        raise ValueError('Expected generated base_link writing data with 26 strokes')
    rclpy.init(args=ros_args)
    node = WritingPreview(data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__ == '__main__':
    main()
