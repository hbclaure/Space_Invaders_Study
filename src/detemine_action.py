#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import json

def action_from_game_state(game_state):
    ai_position = game_state["ai_position"]
    if ai_position < 400:
        robot_action = "hi"
    else:
        robot_action = "right"

    return robot_action

class DetermineAction():
    '''
    ROS node
    '''
    def __init__(self):
        # Initialize the node
        print("initializing determine robot action node")
        rospy.init_node('determine_robot_action')
        
        # Subscribers
        rospy.Subscriber('space_invaders/game/game_state', String, self.determine_action_cb)

        # Publishers 
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)

    def determine_action_cb(self,game_state):
        ## To do
        ## Read game state
        ## Do logic to figure out move
        ## Publish move
        
        game_state_dict = json.loads(game_state.data)
        robot_action = action_from_game_state(game_state_dict)
        self.robot_action_pub.publish(robot_action)

    def run(self):
        pass

if __name__ == '__main__':
    try:
        determine_action = DetermineAction()
        while not rospy.is_shutdown():
            determine_action.run()

    except rospy.ROSInterruptException:
        pass