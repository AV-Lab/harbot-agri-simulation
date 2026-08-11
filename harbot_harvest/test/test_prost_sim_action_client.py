#!/usr/bin/env python3

import rospy

from rbkairos_etf_services.srv import ActionServer


# Edit these three values for quick manual tests.
ROBOT_NS = "/robot"
ACTION_ID = "grasp_fruit"
INPUT = [1, 1, 0, 1, 0, 0]
TIMEOUT = 120.0

# Examples:
# navigate to a1:       ACTION_ID = "navigate",    INPUT = [1, 0, 0, 0, 0, 0]
# grasp orange1.1.L.T:  ACTION_ID = "grasp_fruit", INPUT = [1, 1, 0, 1, 0, 0]
# load orange2.7.R.B:   ACTION_ID = "load_to_bin", INPUT = [2, 7, 1, 0, 0, 0]
# unload container:     ACTION_ID = "unload",      INPUT = [0, 0, 0, 0, 0, 0]


def main():
    rospy.init_node("test_prost_sim_action_client", anonymous=True)

    service_name = "{}/sim_action_server".format(ROBOT_NS.rstrip("/"))
    rospy.loginfo("Waiting for %s", service_name)
    rospy.wait_for_service(service_name)

    call_action = rospy.ServiceProxy(service_name, ActionServer)
    response = call_action(ACTION_ID, INPUT, TIMEOUT)

    rospy.loginfo("success=%s", response.success)
    rospy.loginfo("message=%s", response.message)


if __name__ == "__main__":
    main()
