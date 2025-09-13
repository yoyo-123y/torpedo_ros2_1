import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
import math
import random as rm
class hunterTurtle(Node):
    def __init__(self, turtle_names):
        super().__init__("hunterTurtle")
        self.turtles_poses = {}
        self.hunted_turtle=''
        self.hunted_distance = float("inf")
        topic = f"/turtle1/cmd_vel"
        self.cli = self.create_client(Spawn, 'spawn')
        self.kill = self.create_client(Kill, 'kill')
        self.hunter_publisher = self.create_publisher(Twist, topic, 10)
        self.turtles_poses = {name: None for name in turtle_names}
        for name in turtle_names:
            self.create_subscription(Pose, f"/{name}/pose",lambda msg, n=name: self.update_pose(n, msg),10)
        self.create_timer(0.5, self.move_turtles)
    def update_pose(self,name,pos):
        self.turtles_poses[name]=pos        
    def hunt_mode(self):
        turtle_angle=0.0
        main_turtle_pos = self.turtles_poses['turtle1']
        other_turtles_pos = self.turtles_poses[self.hunted_turtle]
        dy = other_turtles_pos.y - main_turtle_pos.y
        dx = other_turtles_pos.x - main_turtle_pos.x
        turtle_angle = math.atan2(dy, dx)
        angle_diff = turtle_angle - main_turtle_pos.theta
        return angle_diff
    def chooseTurtle(self):
        main_turtle_pos = self.turtles_poses['turtle1']
        self.hunted_distance = float("inf")
        self.hunted_turtle = '' 
        for other_turtles_name, other_turtles_pos in self.turtles_poses.items():
            if other_turtles_name == 'turtle1' or other_turtles_pos==None:
                continue
            distance = ((main_turtle_pos.x - other_turtles_pos.x)**2 + (main_turtle_pos.y - other_turtles_pos.y)**2) ** 0.5
            if(distance<self.hunted_distance):
                self.hunted_turtle=other_turtles_name
                self.hunted_distance=distance
    def check_collision(self):
        main_turtle_pos = self.turtles_poses['turtle1']
        for other_turtles_name, other_turtles_pos in self.turtles_poses.items():
            if other_turtles_name == 'turtle1':
                continue
            distance = ((main_turtle_pos.x - other_turtles_pos.x)**2 + (main_turtle_pos.y - other_turtles_pos.y)**2) ** 0.5
            if distance < 0.5:
                return other_turtles_name
        return False
    # def spawn_turtle(self,name:str):
    #     req = Spawn.Request()
    #     while True:
    #         x_d=float(rm.randint(1,10))
    #         y_d=float(rm.randint(1,10))
    #         for i in self.turtles.values():
    #             if(x_d==i[0] and y_d==i[1]):
    #                 break
    #         if((x_d!=5 and y_d!=5)):
    #             break
    #     req.x = x_d
    #     req.y = y_d
    #     req.theta = rm.random()*math.pi*2
    #     self.turtles[name]=[x_d,y_d,req.theta]
    #     req.name = name
    #     future = self.cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, future)
    #     self.get_logger().info(f"Turtle spawned: {future.result().name}")
    def killTurtle(self,name):
        req = Kill.Request()
        req.name=name
        future = self.kill.call_async(req)
        rclpy.spin_until_future_complete(self, future)
    def move_turtles(self):
        pose = self.turtles_poses['turtle1']
        msg = Twist()
        kill_var=self.check_collision()
        if(kill_var!=False):
            self.killTurtle(kill_var)
            self.hunted_turtle = ''
            self.hunted_distance = float("inf")
            self.turtles_poses.pop(kill_var, None)
            self.chooseTurtle()
        msg.linear.x = 1.3
        if not self.hunted_turtle:
            self.chooseTurtle()
        msg.angular.z = self.hunt_mode()
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
