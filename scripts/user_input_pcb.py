#!/usr/bin/env python3
import re
import rospy
import serial
from std_msgs.msg import String
from typing import List

from sensor_interface.msg import UserInput


class UserInputInterface:
    publisher = None
    subscriber = None
    # dispensing_in_progress = False
    serial = None
    rate = None

    def __init__(self, device="/dev/ttyACM0", publish_rate=500) -> None:
        rospy.init_node("user_input_node")
        self.publisher = rospy.Publisher("user_input", UserInput, queue_size=1)
        self.subscriber = rospy.Subscriber(
            "dispensing_update", String, self.update_dispensing_state
        )

        self.serial = serial.Serial(device, 9600, timeout=1)
        self.rate = rospy.Rate(publish_rate)

    def update_dispensing_state(self, dispensing_update: String):
        self.serial.write(dispensing_update.data.encode())

    def read_user_input(self):
        count = 0
        while not rospy.is_shutdown():
            line = self.serial.readline()  # read a '\n' terminated line
            if type(line) is bytes:
                data = line.decode("utf-8")
                if len(data) == 0:
                    continue
                # message pipeline to start dispensing
                send_msg = UserInput()
                send_msg.header.seq = count
                send_msg.header.stamp = rospy.Time.now()
                send_msg.ingredient = data[: data.index("#")]
                send_msg.quantity = int(data[data.index("#") + 1 :])
                count += 1
                self.publisher.publish(send_msg)
                self.rate.sleep()
            else:
                rospy.logwarn("Garbage value recieved from the weighing scale")


if __name__ == "__main__":
    try:
        inputObj = UserInputInterface()
        # inputObj.read_user_input(device=rospy.get_param('device'))
        inputObj.read_user_input()
    except rospy.ROSInterruptException:
        pass
