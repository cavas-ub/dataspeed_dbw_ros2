#!/usr/bin/env python

# Software License Agreement (Proprietary and Confidential)
#
# Copyright (c) 2019, Dataspeed Inc.
# All rights reserved.
#
# NOTICE:  All information contained herein is, and remains the property of
# Dataspeed Inc. The intellectual and technical concepts contained herein are
# proprietary to Dataspeed Inc. and may be covered by U.S. and Foreign Patents,
# patents in process, and are protected by trade secret or copyright law.
# Dissemination of this information or reproduction of this material is strictly
# forbidden unless prior written permission is obtained from Dataspeed Inc.

import rospy

from dbw_fca_msgs.msg import Misc2Report, MiscCmd
from std_msgs.msg import Bool, Empty

class MiscHvacCmdTest:

    def __init__(self):
        rospy.init_node('vent')

        self.initializing = True

        self.i = 0
        self.start = 0
        self.end = 100000

        # resolution is 20ms
        self.duration = rospy.Duration(0.01)

        self.dbw_enabled = False
        self.msg_misc2_report = Misc2Report()

        rospy.loginfo('Sending vent command every ' + str(self.duration.to_sec()) + ' seconds from ' + str(self.start) + ' to ' + str(self.end) )
 
        # Publishers
        global enable_pub, vent_pub, misc_pub
        enable_pub   = rospy.Publisher('/vehicle/enable',       Empty,       queue_size=1)
        misc_pub     = rospy.Publisher('/vehicle/misc_cmd',     MiscCmd,     queue_size=1)

        # Subscribers
        # DBW should be enabled
        rospy.Subscriber('/vehicle/dbw_enabled',      Bool,           self.recv_dbw_enabled)
        rospy.Subscriber('/vehicle/misc_2_report',    Misc2Report,    self.recv_misc2_report)

        # To publish /vehicle/enable message to enable dbw system
        rospy.Timer(rospy.Duration(0.5), self.initialize, oneshot=True)


    def initialize(self, event):
        rospy.loginfo('Initialize: Start to enable DBW.')
        timeout = 10.0
        wait_time = 0.0
        
        # For test
        self.dbw_enabled = True

        while not self.dbw_enabled and wait_time < timeout:
            enable_pub.publish()
            rospy.sleep(0.01)
            wait_time += 0.01        

        while True:
            if self.dbw_enabled == True:
                rospy.loginfo('Initialize: DBW is enabled. Start timer_process')
                self.initializing = False
                rospy.Timer(rospy.Duration(0.02), self.timer_process) 
                break

        rospy.Timer(rospy.Duration(0.02), self.timer_process)              

    def timer_process(self, event):

        if self.initializing:
            rospy.signal_shutdown('')
            return

        if not self.dbw_enabled:
            rospy.logerr("No new messages on topic '/vehicle/enable'. Enable DBW first.")
        else:
            self.i += 1
            msg = MiscCmd()
            msg.ft_drv_temp.value = 70
            msg.ft_psg_temp.value = 67
            msg.ft_fan_speed.value = 5
            msg.vent_mode.value = msg.vent_mode.FLOOR
            msg.sync.cmd = msg.sync.cmd.OFF
            msg.hsw.cmd = msg.hsw.cmd.ON
            msg.fl_heated_seat.cmd = msg.fl_heated_seat.cmd.HI
            misc_pub.publish(msg)
            rospy.loginfo('hvac commands were sent.')

    def recv_misc2_report(self, msg):
        self.msg_misc2_report = msg
        self.msg_misc2_report_ready = True

    def recv_dbw_enabled(self, msg):
        rospy.loginfo('DBW is enabled')
        self.dbw_enabled = msg.data            

    def shutdown_handler(self):
        rospy.loginfo('shutdown_handler')

if __name__ == '__main__':
    try:
        node = MiscHvacCmdTest()
        rospy.on_shutdown(node.shutdown_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
