#!/usr/bin/env python3
"""Load route graph from Sidecar-published topic into Nav2 route_server.

Subscribes to route_graph_json (std_msgs/String, Transient Local QoS).
On receiving a GeoJSON string, saves it to a file and calls
route_server's SetRouteGraph service to load it.
"""

import json
import os
import tempfile

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point as GeoPoint
from nav2_msgs.srv import SetRouteGraph
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class RouteGraphLoader(LifecycleNode):

    def __init__(self):
        super().__init__('route_graph_loader')

    def on_configure(self, state: LifecycleState):
        self.declare_parameter('graph_save_path', '/tmp/route_graph.geojson')
        self.declare_parameter('set_graph_service', 'route_server/set_route_graph')
        self.declare_parameter('service_timeout', 30.0)

        self._save_path = self.get_parameter('graph_save_path').value
        self._svc_name = self.get_parameter('set_graph_service').value
        self._timeout = self.get_parameter('service_timeout').value
        self._pending_geojson = None
        self._loading = False

        self._cli = self.create_client(SetRouteGraph, self._svc_name)

        self.get_logger().info(
            f'Route graph loader configured: service={self._svc_name}')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.declare_parameter('route_graph_topic', 'route_graph_json')
        self.declare_parameter('route_graph_markers_topic', 'route_graph/markers')
        graph_topic = self.get_parameter('route_graph_topic').value
        markers_topic = self.get_parameter('route_graph_markers_topic').value
        self._graph_topic = graph_topic

        self._sub = self.create_subscription(
            String, graph_topic, self._on_graph, qos)

        marker_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._marker_pub = self.create_publisher(
            MarkerArray, markers_topic, marker_qos)

        self._timer = self.create_timer(2.0, self._wait_service)

        self.get_logger().info(
            f'Ready: waiting for {graph_topic}, service={self._svc_name}')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState):
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        return TransitionCallbackReturn.SUCCESS

    def _wait_service(self):
        if self._pending_geojson is None:
            return
        if not self._cli.service_is_ready():
            self.get_logger().warn(
                f'Service {self._svc_name} not ready, retrying...')
            return
        self._timer.cancel()
        self._load(self._pending_geojson)
        self._pending_geojson = None

    def _on_graph(self, msg: String):
        geojson_str = msg.data.strip()
        if not geojson_str:
            self.get_logger().warn(
                f'Received empty {self._graph_topic} message, ignoring')
            return

        # Validate JSON structure
        try:
            obj = json.loads(geojson_str)
        except json.JSONDecodeError as e:
            self.get_logger().error(
                f'Invalid JSON in {self._graph_topic}: {e}')
            return

        if obj.get('type') != 'FeatureCollection':
            self.get_logger().error(
                f'Expected type=FeatureCollection, got {obj.get("type")}')
            return

        n_nodes = sum(
            1 for f in obj.get('features', [])
            if f.get('geometry', {}).get('type') == 'Point')
        n_edges = sum(
            1 for f in obj.get('features', [])
            if f.get('geometry', {}).get('type') in ('LineString', 'MultiLineString'))
        self.get_logger().info(
            f'Received route graph: {n_nodes} nodes, {n_edges} edges')

        self._publish_markers(obj)

        if self._loading:
            self.get_logger().warn(
                'Graph load in progress, queuing new graph')
            self._pending_geojson = geojson_str
            return

        if self._cli.service_is_ready():
            self._load(geojson_str)
        else:
            self.get_logger().info(
                f'Service {self._svc_name} not ready, will load when available')
            self._pending_geojson = geojson_str
            self._timer.reset()

    def _load(self, geojson_str: str):
        self._loading = True

        # Save to file
        try:
            dir_name = os.path.dirname(self._save_path)
            if dir_name:
                os.makedirs(dir_name, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(
                suffix='.geojson', dir=dir_name or None)
            with os.fdopen(fd, 'w') as f:
                f.write(geojson_str)
            os.replace(tmp_path, self._save_path)
        except OSError as e:
            self._loading = False
            self.get_logger().error(f'Failed to save graph file: {e}')
            return

        self.get_logger().info(f'Graph saved to {self._save_path}')

        # Call SetRouteGraph service
        req = SetRouteGraph.Request()
        req.graph_filepath = self._save_path

        future = self._cli.call_async(req)
        future.add_done_callback(self._on_load_result)

    def _on_load_result(self, future):
        self._loading = False
        try:
            result = future.result()
            if result.success:
                self.get_logger().info('Route graph loaded successfully')
            else:
                self.get_logger().error('Route graph load failed')
        except Exception as e:
            self.get_logger().error(f'SetRouteGraph service call failed: {e}')

        # Process queued graph if any
        if self._pending_geojson is not None:
            pending = self._pending_geojson
            self._pending_geojson = None
            self._load(pending)

    def _publish_markers(self, obj: dict):
        """Publish MarkerArray visualization from parsed GeoJSON."""
        markers = MarkerArray()

        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        mid = 0
        now = self.get_clock().now().to_msg()

        for feature in obj.get('features', []):
            geom = feature.get('geometry', {})
            props = feature.get('properties', {})
            gtype = geom.get('type')

            if gtype == 'Point':
                coords = geom.get('coordinates', [])
                if len(coords) < 2:
                    continue
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = now
                m.ns = 'route_nodes'
                m.id = mid
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(coords[0])
                m.pose.position.y = float(coords[1])
                m.pose.position.z = 0.0
                m.pose.orientation.w = 1.0
                m.scale.x = 0.3
                m.scale.y = 0.3
                m.scale.z = 0.3
                m.color.r = 0.2
                m.color.g = 0.6
                m.color.b = 1.0
                m.color.a = 1.0
                markers.markers.append(m)
                mid += 1

                name = props.get('name')
                if name:
                    t = Marker()
                    t.header.frame_id = 'map'
                    t.header.stamp = now
                    t.ns = 'route_labels'
                    t.id = mid
                    t.type = Marker.TEXT_VIEW_FACING
                    t.action = Marker.ADD
                    t.pose.position.x = float(coords[0])
                    t.pose.position.y = float(coords[1])
                    t.pose.position.z = 0.4
                    t.pose.orientation.w = 1.0
                    t.scale.z = 0.25
                    t.color.r = 1.0
                    t.color.g = 1.0
                    t.color.b = 1.0
                    t.color.a = 1.0
                    t.text = str(name)
                    markers.markers.append(t)
                    mid += 1

            elif gtype == 'LineString':
                coords = geom.get('coordinates', [])
                if len(coords) < 2:
                    continue
                line = Marker()
                line.header.frame_id = 'map'
                line.header.stamp = now
                line.ns = 'route_edges'
                line.id = mid
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.pose.orientation.w = 1.0
                line.scale.x = 0.08
                line.color.r = 0.0
                line.color.g = 1.0
                line.color.b = 0.5
                line.color.a = 0.8
                for c in coords:
                    line.points.append(GeoPoint(
                        x=float(c[0]), y=float(c[1]), z=0.0))
                markers.markers.append(line)
                mid += 1

        self._marker_pub.publish(markers)
        self.get_logger().info(
            f'Published {mid} route graph visualization markers')


def main(args=None):
    rclpy.init(args=args)
    node = RouteGraphLoader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
