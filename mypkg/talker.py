import rclpy
from rclpy.node import Node
from std_msgs.msg import Person

rclpy.init()
node = Node("talker")
pub = node.create_publisher(Person, "person", 10)
n = 0


def cb():
    global n
    msg = Person()
    msg.name = "増山耀一"
    msg.age = n
    #msg.data = n
    pub.publish(msg)
    n += 1


def main():
    node.create_timer(0.5, cb)
    rclpy.spin(node)
