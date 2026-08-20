#!/usr/bin/env python3
"""
Publishes persistent A (start) and B (goal) markers + text labels on
/ab_markers so RViz2 shows them immediately on launch, before arming or
triggering -- independent of Fast-Planner's own drawGoal() (one-shot,
VOLATILE QoS on /planning_vis/trajectory, only visible if RViz2 happens
to be subscribed at the exact moment waypointCallback fires).

A = (0, 0, 5) -- spawn point, hover altitude (fsm/waypoint0_z in
fast_planner_px4.launch.py), not the ground spawn (0,0,0): the mission's
real start-of-travel is the post-climb hover point, and z=0 would render
the marker underground/at takeoff position instead of where cruise
actually begins.
B = (0, 8, 5) -- fsm/waypoint0_{x,y,z} in fast_planner_px4.launch.py,
the actual mission goal.

Publishes on a 1Hz timer (not latched/transient_local -- simplest given
this project's existing pattern of plain reliable QoS elsewhere) so any
RViz2 instance that (re)starts at any point during a flight still picks
these up within ~1s, matching the user's ask ("from the start").
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

A_POS = (0.0, 0.0, 5.0)
B_POS = (0.0, 8.0, 5.0)


class ABMarkers(Node):
    def __init__(self):
        super().__init__('ab_markers')
        self.pub = self.create_publisher(MarkerArray, '/ab_markers', 10)
        self.timer = self.create_timer(1.0, self.tick)

    def tick(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        def sphere(mid, pos, r, g, b):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'ab_points'
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            return m

        def text(mid, pos, label, r, g, b):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'ab_labels'
            m.id = mid
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2] + 0.7)
            m.pose.orientation.w = 1.0
            m.scale.z = 0.6
            m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
            m.text = label
            return m

        arr.markers.append(sphere(0, A_POS, 0.2, 0.9, 0.2))
        arr.markers.append(text(1, A_POS, 'A (start)', 0.2, 0.9, 0.2))
        arr.markers.append(sphere(2, B_POS, 0.9, 0.2, 0.2))
        arr.markers.append(text(3, B_POS, 'B (goal)', 0.9, 0.2, 0.2))

        self.pub.publish(arr)


def main():
    rclpy.init()
    node = ABMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
