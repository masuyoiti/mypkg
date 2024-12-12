import rclpy
from rclpy.node import Node
from person_msgs.msg import Int16


class Talker(Node:)
    def_init_(self):
        super()._init_("Talker")
        self.pub = self.create_publisher(Int16, "countup", 10)
        self.creat_timer(0.5, self.cb)
        self.n = 0


def main():
    rclpy.init()
    node = Talker()
    rclpy.spin(node)

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
