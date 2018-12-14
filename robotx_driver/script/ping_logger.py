#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import subprocess
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('/network_ping', String, queue_size=10)
    rospy.init_node('ping_logger', anonymous=True)
    rate = rospy.Rate(0.1)
    ip_list = ['192.168.1.{0}'.format(subnet) for subnet in [101, 102, 103, 251, 252, 16]]
    while not rospy.is_shutdown():
        out_list = []
        for ip in ip_list:
            try:
                output = subprocess.check_output('ping -c 1 -w 1 {0}'.format(ip),
                        stderr=subprocess.STDOUT, shell=True)
                out = output.split('\n')[1]
            except subprocess.CalledProcessError as exc:
                out = 'failed {0}'.format(ip)
            out_list.append(out)
        pub.publish(','.join(out_list))
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
