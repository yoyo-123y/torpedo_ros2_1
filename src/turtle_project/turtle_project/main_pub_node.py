import rclpy
from rclpy.node import Node
from turtlesim.srv import *
import random as rm
import math
class initiateTurtle(Node):
    def __init__(self):
        super().__init__("initiateTurtle")
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        self.turtles={
            "turtle1":[0,0,0],
            "turtle2":[0,0,0],
            "turtle3":[0,0,0],
            "turtle4":[0,0,0],
            "turtle5":[0,0,0],
        }
        self.spawn_turtle("turtle_1")
        self.spawn_turtle("turtle_2")
        self.spawn_turtle("turtle_3")
        self.spawn_turtle("turtle_4")
        self.spawn_turtle("turtle_5")
    def spawn_turtle(self,name:str):
        req = Spawn.Request()
        while True:
            x_d=float(rm.randint(1,10))
            y_d=float(rm.randint(1,10))
            for i in self.turtles.values():
                if(x_d==i[0] and y_d==i[1]):
                    break
            if((x_d!=5 and y_d!=5)):
                break
        req.x = x_d
        req.y = y_d
        req.theta = rm.random()*math.pi*2
        self.turtles[name]=[x_d,y_d,req.theta]
        req.name = name
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Turtle spawned: {future.result().name}")
def main(args=None):
    rclpy.init(args=args)
    node = initiateTurtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()