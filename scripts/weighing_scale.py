#!/usr/bin/env python3
import re
import rospy
import serial

from sensor_interface.msg import Weight


def read_weight(device="/dev/ttyUSB0", publish_rate=20):
    pub = rospy.Publisher("weighing_scale", Weight, queue_size=10)
    count = 0
    rospy.init_node("weighing_scale")
    rate = rospy.Rate(publish_rate)
    ser = serial.Serial(device, 9600, timeout=1)
    while not rospy.is_shutdown():
        line = ser.readline()  # read a '\n' terminated line
        if type(line) is bytes:
            data = line.decode("utf-8")
            parsed_data = re.findall(r"[-+]?(?:\d*\.\d+|\d+)", data)
            if len(parsed_data) == 0:
                rospy.logerr(
                    f"Invalid value from sensor. Raw value received: {data}"\
                    + "Likely causes include exceeding weighing scale range."
                )
                continue
            send_msg = Weight()
            send_msg.header.seq = count
            send_msg.header.stamp = rospy.Time.now()
            send_msg.weight = float(parsed_data[0])
            if data[0] == '-':
                send_msg.weight *= -1
            count += 1
            pub.publish(send_msg)
            rate.sleep()
        else:
            rospy.logwarn("Garbage value recieved from the weighing scale")


if __name__ == "__main__":
    try:
        read_weight(device=rospy.get_param('device'))
        # read_weight()
    except rospy.ROSInterruptException:
        pass
