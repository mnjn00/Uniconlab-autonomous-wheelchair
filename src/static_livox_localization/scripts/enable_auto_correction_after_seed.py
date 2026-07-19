#!/usr/bin/env python3
"""Enable assisted ICP only after replay delivers an operator seed."""

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import SetBool


class EnableAutoCorrectionAfterSeed:
    def __init__(self):
        self.completed = False
        self.subscriber = rospy.Subscriber(
            "/fast_lio_icp/initialpose",
            PoseWithCovarianceStamped,
            self.seed_callback,
            queue_size=1,
        )

    def seed_callback(self, _message):
        if self.completed:
            return
        try:
            rospy.wait_for_service(
                "/fast_lio_icp/enable_auto_correction", timeout=10.0
            )
            enable = rospy.ServiceProxy(
                "/fast_lio_icp/enable_auto_correction", SetBool
            )
            response = enable(True)
            if response.success:
                self.completed = True
                self.subscriber.unregister()
                rospy.loginfo("Replay auto correction enabled after seed")
            else:
                rospy.logerr("Replay auto correction rejected: %s", response.message)
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logerr("Replay could not enable auto correction: %s", error)


if __name__ == "__main__":
    rospy.init_node("enable_auto_correction_after_seed")
    EnableAutoCorrectionAfterSeed()
    rospy.spin()
