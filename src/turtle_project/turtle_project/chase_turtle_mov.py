import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
class TurtleController(Node):
    def __init__(self, turtle_names):
        super().__init__("turtle_controller")
        self.turtles_publishers = {}
        self.turtles_poses = {}
        for name in turtle_names:
            topic = f"/{name}/cmd_vel"
            self.turtles_publishers[name] = self.create_publisher(Twist, topic, 10)
            self.turtles_poses = {name: None for name in turtle_names}
            self.create_subscription(Pose, f"/{name}/pose",lambda msg, n=name: self.update_pose(n, msg),10)

        self.create_timer(0.5, self.move_turtles)
    def update_pose(self,name,pos):
        self.turtles_poses[name]=pos
    def check_collision(self, name):
        main_turtle_pos = self.turtles_poses[name]
        for other_turtles_name, other_turtles_pos in self.turtles_poses.items():
            if other_turtles_name == name:
                continue
            distance = ((main_turtle_pos.x - other_turtles_pos.x)**2 + (main_turtle_pos.y - other_turtles_pos.y)**2) ** 0.5
            if distance < 0.5:
                return True
        return False
    def move_turtles(self):
        for name, pub in self.turtles_publishers.items():
            pose = self.turtles_poses[name]
            if pose is None:
                continue
            msg = Twist()
            if (self.check_collision(name)):
                msg.angular.z = math.pi
            msg.linear.x = 1.0
            if pose.x <= 0 or pose.x >= 10.5 or pose.y <= 0.5 or pose.y >= 10.5:
                msg.angular.z = math.pi / 2 
            pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    turtles = ["turtle_1", "turtle_2", "turtle_3", "turtle_4", "turtle_5"]
    node = TurtleController(turtles)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
