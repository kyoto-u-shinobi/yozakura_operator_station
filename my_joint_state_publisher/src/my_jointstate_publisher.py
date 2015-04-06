#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from arm_jointstate_manager import ArmJointStateManager
from body_jointstate_manager import BodyJointStateManager

if __name__ == '__main__':
    rospy.init_node('my_jointstate_manager', anonymous=True)

    jointstate_msg = JointState()

    arm_js_mgr = ArmJointStateManager()
    arm_js_mgr.run()

    body_js_mgr = BodyJointStateManager()
    body_js_mgr.run()

    try:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rate_mgr = rospy.Rate(10)  # 10hz
        print 'Run!'
        while not rospy.is_shutdown():
            jointstate_msg.header.stamp = rospy.Time.now()
            jointstate_msg.name = body_js_mgr.get_jointstate().name + arm_js_mgr.get_jointstate().name
            jointstate_msg.position = body_js_mgr.get_jointstate().position + arm_js_mgr.get_jointstate().position
            jointstate_msg.velocity = []
            jointstate_msg.effort = []

            pub.publish(jointstate_msg)
            rate_mgr.sleep()  # this has rospy.spin()
    except rospy.ROSInterruptException:
        pass
