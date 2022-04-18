#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import WrenchStamped


class ForceTorquePublisher(object):
    GRAVITY = 9.820665
    MOMENT_ARM_LENGTH = 0.15

    def __init__(self):
        self.value = 0

        rospy.init_node("force_torque_publisher")

        self.pub = rospy.Publisher("/force_torque_weight", Float64, queue_size=10)
        rospy.Subscriber("/wrench", WrenchStamped, self.update_value)

    def update_value(self, msg):
        # self.value = msg.wrench.torque.x
        torque = msg.wrench.torque.x
        mass = torque / (self.GRAVITY)
        mass = (
            mass / self.MOMENT_ARM_LENGTH
        )  # Container specific could be ( 15cm away from tool flange) OREGANO
        # mass = mass/0.155 #Container specific could be ( 15cm away from tool flange) PULSE
        self.pub.publish(mass)

    def run(self):
        r = rospy.Rate(1000)
        while not rospy.is_shutdown():
            # self.pub.publish(self.value)
            r.sleep()


if __name__ == "__main__":
    ftpublisher = ForceTorquePublisher()
    try:
        ftpublisher.run()
    except rospy.ROSInterruptException:
        pass
