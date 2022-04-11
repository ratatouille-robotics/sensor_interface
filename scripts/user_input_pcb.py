#!/usr/bin/env python3
import re
import rospy
import serial
from std_msgs.msg import Bool

from sensor_interface.msg import UserInput


class UserInputInterface:
    publisher = None
    subscriber = None
    dispensing_in_progress = False
    serial = None
    rate = None

    def __init__(self, device="/dev/ttyACM0", publish_rate=500) -> None:
        rospy.init_node("user_input_node")
        self.publisher = rospy.Publisher("user_input", UserInput, queue_size=10)
        self.subscriber = rospy.Subscriber(
            "/dispensing_progress", Bool, self.update_dispensing_state
        )
        self.serial = serial.Serial(device, 9600, timeout=1)
        self.rate = rospy.Rate(publish_rate)

        # self.read_user_input()

    def update_dispensing_state(self, progressMsg):
        self.dispensing_in_progress = progressMsg.data
        # print(self.dispensing_in_progress)
        if self.dispensing_in_progress:
            self.serial.write(b"t")
        else:
            self.serial.write(b"f")
        # Serial Write to arduino

    def read_user_input(self):
        count = 0
        while not rospy.is_shutdown():
            line = self.serial.readline()  # read a '\n' terminated line
            if type(line) is bytes:
                data = line.decode("utf-8")
                # print(f"Received = {data}")
                # parsed_data = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", data)
                # print(f"Parsed = {parsed_data}")
                if len(data) == 0:
                    # rospy.logerr(
                    #     f"Invalid value from sensor. Raw value received: {data}"
                    # )
                    # print("Stopping serial read")
                    # break
                    continue
                # message pipeline to start dispensing
                send_msg = UserInput()
                send_msg.header.seq = count
                send_msg.header.stamp = rospy.Time.now()
                send_msg.ingredient = data[: data.index("#")]
                send_msg.quantity = int(data[data.index("#") + 1 :])
                # print(f"Name: {send_msg.ingredient}")
                # print(f"Quantity: {send_msg.quantity}")

                # write read input

                # if parsed_data is of form
                # self.dispensing_in_progress = True
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
