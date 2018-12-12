#!/usr/bin/env python

import os
import datetime
import rospy
from std_msgs.msg import Empty

def ping(address, timeout=5, attempts=4):
    failed_attempts = 0
    for i in range(0, attempts):
        result = os.system('/bin/ping -c 1 -w %i %s > /dev/null 2>&1' % (timeout, address))
        if result != 0:
            failed_attempts += 1

    if failed_attempts == attempts:
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M')
        print("%s 'ping %s' failed attempts: %i.\n" % (now, address, failed_attempts))
        
        return False
    else:
        return True
    

def watchdog():
    rospy.init_node('watchdog', anonymous=True)
    rate = rospy.Rate(10)
    pub = rospy.Publisher('/kill', Empty, queue_size=10)
    while not rospy.is_shutdown():
        if not ping("192.168.1.16"):
            pub.publish()
        rate.sleep()

if __name__ == '__main__':
    try:
        watchdog()
    except rospy.ROSInterruptException:
        pass
    
