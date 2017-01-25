#!/usr/bin/env python
import roslib;

roslib.load_manifest('automow_node')

import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from automow_node.msg import Status

global diag_publisher

BATTERY_STATES = {0: "Discharging",
                  1: "Recovery Charging",
                  2: "Charging",
                  3: "Trickle Charging",
                  4: "Critical Discharging",
                  5: "Error"}


def status_cb(msg):
    global BATTERY_STATES
    global diag_publisher
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = rospy.Time.now()
    diag_msg.status = []

    batt_status = DiagnosticStatus()
    batt_status.name = "Battery Status"
    batt_status.hardware_id = "robot"
    batt_status.values = []
    state = BATTERY_STATES[msg.battery_state]
    batt_status.values.append(KeyValue(key="State",
                                       value=state))
    batt_status.values.append(KeyValue(key="Charge",
                                       value="{:.0%}".format(msg.charge / 100.0)))
    batt_status.values.append(KeyValue(key="Voltage",
                                       value="%3.2f V" % (msg.voltage / 1000.0)))
    batt_status.values.append(KeyValue(key="Battery Current",
                                       value="%3.2f A" % (msg.current / 1000.0)))
    diag_msg.status.append(batt_status)
    if msg.battery_state >= 4:
        batt_status.level = batt_status.ERROR
    else:
        batt_status.level = batt_status.OK
    batt_status.message = state
    diag_publisher.publish(diag_msg)


def diagnostics_bridge():
    global diag_publisher
    rospy.init_node('diagnostics_bridge', anonymous=True)
    rospy.Subscriber("/robot/status", Status, status_cb)
    diag_publisher = rospy.Publisher("/diagnostics", DiagnosticArray, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    diagnostics_bridge()
