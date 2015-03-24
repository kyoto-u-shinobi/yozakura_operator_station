#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from arm_jointstate_manager import ArmJointStateManager
from body_jointstate_manager import BodyJointStateManager

if __name__ == '__main__':
    rospy.init_node('my_jointstate_manager', anonymous=True)

    joint_state_msg = JointState()

    arm_js_mgr = ArmJointStateManager()
    arm_js_mgr.run()

    body_js_mgr = BodyJointStateManager()
    body_js_mgr.run()

    try:
        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        ros_looprate_mgr = rospy.Rate(10)  # 10hz
        print 'Run!'
        while not rospy.is_shutdown():
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = body_js_mgr.get_jointstate_name() + arm_js_mgr.get_jointstate_name()
            joint_state_msg.position = body_js_mgr.get_jointstate_position() + arm_js_mgr.get_jointstate_position()
            joint_state_msg.velocity = []
            joint_state_msg.effort = []

            pub.publish(joint_state_msg)
            ros_looprate_mgr.sleep()  # this has rospy.spin()
    except rospy.ROSInterruptException:
        pass
