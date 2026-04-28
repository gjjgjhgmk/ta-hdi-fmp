import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray

from ta_hdi_fmp_msgs.srv import PlanPath2D
from ta_hdi_fmp_core import plan_path, PlannerConfig


class PlannerNode(Node):
    def __init__(self):
        super().__init__('ta_hdi_fmp_planner')

        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        self.cfg = PlannerConfig()

        self.path_pub = self.create_publisher(Path, '/ta_hdi_fmp/planned_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/ta_hdi_fmp/debug_markers', 10)
        self.srv = self.create_service(PlanPath2D, '/ta_hdi_fmp/plan_path', self.handle_plan)

        self.get_logger().info('Planner service ready: /ta_hdi_fmp/plan_path')

    def handle_plan(self, request, response):
        start = [request.start.x, request.start.y]
        goal = [request.goal.x, request.goal.y]

        obstacles = []
        for o in request.obstacles:
            obstacles.append([float(o.cx), float(o.cy), float(o.radius), float(o.width), float(o.theta), float(o.type)])

        result = plan_path(start, goal, obstacles, self.cfg)

        response.success = bool(result['success'])
        response.min_dist = float(result['metrics']['min_dist'])
        response.kappa_p99 = float(result['metrics']['kappa_max'])
        response.jerk_rms = float(result['metrics']['jerk_rms'])
        response.status = str(result['debug']['status'])

        response.path = self._to_path_msg(result['path_xy'])
        self.path_pub.publish(response.path)
        self.marker_pub.publish(self._obstacle_markers(request.obstacles))

        return response

    def _to_path_msg(self, path_xy):
        msg = Path()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        for xy in path_xy:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(xy[0])
            ps.pose.position.y = float(xy[1])
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg

    def _obstacle_markers(self, obstacles):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, o in enumerate(obstacles):
            mk = Marker()
            mk.header.frame_id = self.frame_id
            mk.header.stamp = now
            mk.ns = 'obstacles'
            mk.id = i
            mk.action = Marker.ADD
            mk.pose.position.x = float(o.cx)
            mk.pose.position.y = float(o.cy)
            mk.pose.position.z = 0.05
            mk.pose.orientation.w = 1.0
            mk.color.r = 1.0
            mk.color.g = 0.4
            mk.color.b = 0.4
            mk.color.a = 0.8
            if int(o.type) == 1:
                mk.type = Marker.CYLINDER
                mk.scale.x = 2.0 * float(o.radius)
                mk.scale.y = 2.0 * float(o.radius)
                mk.scale.z = 0.1
            else:
                mk.type = Marker.CUBE
                mk.scale.x = float(o.width)
                mk.scale.y = float(o.height)
                mk.scale.z = 0.1
            ma.markers.append(mk)
        return ma


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
