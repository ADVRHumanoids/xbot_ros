#!/usr/bin/env python
import rospy
from std_msgs.msg import String

import os
import signal
import subprocess
import time

def callback(data):
    my_env = os.environ.copy()
    subprocess.Popen(["espeak", "-v", "mb-us2", "-s", "120", data.data], env=my_env)
    print(data.data)
    time.sleep(1)

def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("cartesian/audio", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
