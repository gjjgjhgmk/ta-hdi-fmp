import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry


class MobileAdapterNode(Node):
    def __init__(self):
        super().__init__('ta_hdi_fmp_mobile_adapter')

        self.declare_parameter('lookahead_dist', 0.6)
        self.declare_parameter('v_max', 0.25)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('timeout_sec', 5.0)

        self.lookahead = float(self.get_parameter('lookahead_dist').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.w_max = float(self.get_parameter('w_max').value)
        self.goal_tol = float(self.get_parameter('goal_tolerance').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.path = []
        self.last_path_t = self.get_clock().now()
        self.pose = None  # (x,y,yaw)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/ta_hdi_fmp/planned_path', self.on_path, 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.timer = self.create_timer(0.05, self.on_timer)

    def on_path(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.last_path_t = self.get_clock().now()

    def on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (x, y, yaw)

    def on_timer(self):
        if self.pose is None or len(self.path) == 0:
            self.publish_stop()
            return

        age = (self.get_clock().now() - self.last_path_t).nanoseconds * 1e-9
        if age > self.timeout_sec:
            self.publish_stop()
            return

        x, y, yaw = self.pose
        gx, gy = self.path[-1]
        dist_goal = math.hypot(gx - x, gy - y)
        if dist_goal < self.goal_tol:
            self.publish_stop()
            return

        tx, ty = self.find_target(x, y)
        alpha = math.atan2(ty - y, tx - x) - yaw
        while alpha > math.pi:
            alpha -= 2.0 * math.pi
        while alpha < -math.pi:
            alpha += 2.0 * math.pi

        v = min(self.v_max, 0.8 * dist_goal)
        w = 2.0 * v * math.sin(alpha) / max(self.lookahead, 1e-6)
        w = max(-self.w_max, min(self.w_max, w))

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)

    def find_target(self, x, y):
        for px, py in self.path:
            if math.hypot(px - x, py - y) >= self.lookahead:
                return px, py
        return self.path[-1]

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MobileAdapterNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
