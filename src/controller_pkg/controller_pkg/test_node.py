from cgi import test
from tracked_object_msgs.msg import TrackedObjectArray, TrackedObject
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class TestNode(Node):

    def __init__(self):

        super().__init__("test")

        self.pub = self.create_publisher(TrackedObjectArray, "/tracked_objects", 10)
        self.sub = self.create_subscription(Int8, "test_topic", self.callback, 10)

        self.timer = self.create_timer(0.1, self.run)
        self.dist = 0.0
        self.dist_y = 0.0
        self.switched = 0


    def callback(self, msg: Int8):
        
        self.switched = msg.data


    def run(self):

        # first object
        object = TrackedObject()
        object.object_pose_m_quat.position.y = 5.0 - self.dist_y
        object.object_pose_m_quat.position.x = 50.0 - self.dist

        object.object_velocity_mps_radps.linear.x = 0.0
        object.object_velocity_mps_radps.linear.y = 0.0
        object.object_velocity_mps_radps.linear.z = 0.0
        object.object_type = 20000


        # second object
        object2 = TrackedObject()
        object2.object_pose_m_quat.position.y = 5.0
        object2.object_pose_m_quat.position.x = 30.0 - self.dist

        object2.object_velocity_mps_radps.linear.x = 0.0
        object2.object_velocity_mps_radps.linear.y = 0.0
        object2.object_velocity_mps_radps.linear.z = 0.0
        object2.object_type = 20000

        if self.switched == 1:
            # switched right
            object.object_pose_m_quat.position.y = 5.0
            object2.object_pose_m_quat.position.y = 15.0

        elif self.switched == 2:
            # switched left
            object.object_pose_m_quat.position.y = 5.0
            object2.object_pose_m_quat.position.y = 10.0


        # Object list
        objects = TrackedObjectArray()
        objects.tracked_objects = [object, object2]

        self.pub.publish(objects)
        self.dist += 0.1
        self.dist_y += 0.01

def main(args=None):
    rclpy.init(args=args)

    test = TestNode()

    rclpy.spin(test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()