#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from arm_jointstate_manager import ArmJointStateManager
from body_jointstate_manager import BodyJointStateManager

if __name__ == '__main__':
    rospy.init_node('jointstate_manager', anonymous=True)

    yozakura_jointstate = JointState()

    arm_js_mgr = ArmJointStateManager()
    arm_js_mgr.sub()

    body_js_mgr = BodyJointStateManager()
#    body_js_mgr.run()

    try:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rate_mgr = rospy.Rate(10)  # 10hz
        print 'Run!'
        while not rospy.is_shutdown():
            yozakura_jointstate.header.stamp = rospy.Time.now()
            yozakura_jointstate.name = body_js_mgr.get_jointstate().name + arm_js_mgr.get_jointstate().name
            yozakura_jointstate.position = body_js_mgr.get_jointstate().position + arm_js_mgr.get_jointstate().position
            yozakura_jointstate.velocity = []
            yozakura_jointstate.effort = []

            pub.publish(yozakura_jointstate)
            rate_mgr.sleep()  # this has rospy.spin()
    except rospy.ROSInterruptException:
        pass
