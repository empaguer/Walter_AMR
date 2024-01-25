#!/usr/bin/env python
import rospy
from pedsim_msgs.msg import AgentState, AgentStates
from leg_tracker.msg import PersonArray


class LegPedsimPublisher:
    def __init__(self):

        # PUBLISHER
        self.pedsim_pub = rospy.Publisher(
            "/pedsim_simulator/simulated_agents", AgentStates, queue_size=5
        )

        # SUBSCRIBER
        self.legs_sub = rospy.Subscriber("/people_tracked", PersonArray, self.legs_cb)

        self.detected_legs = PersonArray()

    def legs_cb(self, msg):
        # print("msg gotten")
        self.detected_legs = msg.people
        agent_list = []

        for i in range(0, len(self.detected_legs)):
            agent_state = AgentState()
            agent_state.id = i
            agent_state.pose.position.x = self.detected_legs[i].pose.position.x
            agent_state.pose.position.y = self.detected_legs[i].pose.position.y
            agent_state.pose.position.z = 0

            agent_state.pose.orientation.x = self.detected_legs[i].pose.orientation.x
            agent_state.pose.orientation.y = self.detected_legs[i].pose.orientation.y
            agent_state.pose.orientation.z = self.detected_legs[i].pose.orientation.z
            agent_state.pose.orientation.w = self.detected_legs[i].pose.orientation.w

            agent_state.header.stamp = rospy.Time.now()
            agent_state.header.frame_id = "map"

            agent_list.append(agent_state)

        # print(agent_list)

        agent_states = AgentStates()
        agent_states.agent_states = agent_list
        agent_states.header.stamp = rospy.Time.now()
        agent_states.header.frame_id = "map"

        self.pedsim_pub.publish(agent_states)

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.5)


if __name__ == "__main__":

    rospy.init_node("leg_pedsim_node")

    print("starting node")
    leg_pedsim_publisher = LegPedsimPublisher()
    leg_pedsim_publisher.run()