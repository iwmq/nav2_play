import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


class Nav2PlayNode(Node):
    def __init__(self):
        super(Nav2PlayNode, self).__init__("nav2_play_node")
        self.get_logger().info("Send a list of waypoints")
    
    def send_waypoints(self):
        coords = [
            (-2.0, -1.0, 0.0),
            (1.6, 0.5, 0.0),
            (0.4, -1.7, 0.0),
            (-0.8, 1.9, 0.0)
        ]
        poses = []
        for x, y, z in coords:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z
            poses.append(pose)
        
        goal = FollowWaypoints.Goal()
        goal.poses = poses
        
        ac = ActionClient(self, FollowWaypoints, "follow_waypoints")
        ac.send_goal_async(goal)


def main(arg=None):
    rclpy.init(args=arg)
    node = Nav2PlayNode()
    try:
        node.send_waypoints()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()