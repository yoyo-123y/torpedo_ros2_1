import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
class hunterTurtle(Node):
    def __init__(self, turtle_names):
        super().__init__("hunterTurtle")
        self.turtles_poses = {}
        self.hunted_turtle=''
        self.hunted_distance=0
        self.turtle_angle=0
        topic = f"/turtle1/cmd_vel"
        self.hunter_publisher = self.create_publisher(Twist, topic, 10)
        for name in turtle_names:
            self.turtles_poses = {name: None for name in turtle_names}
            self.create_subscription(Pose, f"/{name}/pose",lambda msg, n=name: self.update_pose(n, msg),10)

        self.create_timer(0.1, self.move_turtles)
    def update_pose(self,name,pos):
        self.turtles_poses[name]=pos
    def hunt_mode(self):
        main_turtle_pos = self.turtles_poses['turtle1']
        for other_turtles_name, other_turtles_pos in self.turtles_poses.items():
            if other_turtles_name == 'turtle1':
                continue
            distance = ((main_turtle_pos.x - other_turtles_pos.x)**2 + (main_turtle_pos.y - other_turtles_pos.y)**2) ** 0.5
            if(distance<self.hunted_distance):
                self.hunted_turtle='turtle1'
                self.hunted_distance=distance
                self.turtle_angle=math.atan((main_turtle_pos.y - other_turtles_pos.y)/(main_turtle_pos.x - other_turtles_pos.x))
                return self.turtle_angle
            
    def move_turtles(self):
        pose = self.turtles_poses['turtle1']
        msg = Twist()
        msg.angular.z = self.hunt_mode()
        msg.linear.x = 1.0
        if pose.x <= 0 or pose.x >= 10.5 or pose.y <= 0.5 or pose.y >= 10.5:
            msg.angular.z = math.pi / 2 
        self.hunter_publisher.publish(msg)
        self.get_logger().info('turtle1')

def main(args=None):
    rclpy.init(args=args)
    turtles = ["turtle1","turtle_1", "turtle_2", "turtle_3", "turtle_4", "turtle_5"]
    node = hunterTurtle(turtles)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
