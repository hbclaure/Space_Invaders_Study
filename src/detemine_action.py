#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import json



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
        rospy.Subscriber('space_invaders/game/game_condition', String, self.game_condition_cb)

        # Publishers 
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)

        # Attributes
        self.game_condition = None
        self.asked_feedback = False
        self.condition_met = False

    def determine_action_cb(self,game_state):
        try:
            game_state_dict = json.loads(game_state.data)
            robot_action = self.action_from_game_state(game_state_dict)
            self.robot_action_pub.publish(robot_action)
            self.robot_action_pub.publish(robot_action)
        except ValueError:
            game_state = game_state.data
            if game_state == "game_over":
                self.robot_action_pub.publish("sleep")
        

    def game_condition_cb(self,msg):
        self.game_condition = msg.data
        self.asked_feedback = False
        self.condition_met = False

    def run(self):
        pass

    def action_from_game_state(self,game_state):
        enemies_left_positions = game_state['enemies_left_positions']
        enemies_right_positions = game_state['enemies_right_positions']
        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        if self.game_condition in ['A','B'] and not(self.asked_feedback):
            # asking for feedback before crossing to help
            if num_left_enemies <= 75 or num_right_enemies <= 75:
                robot_action = "ask_for_feedback"
                self.asked_feedback = True
                print("Ask for feedback: ", self.game_condition)
                return robot_action
        elif self.game_condition in ['C','D'] and not(self.asked_feedback): 
            # after for feedback after crossing to help
            if num_left_enemies <= 50 and game_state["ai_position"] > 400:
                robot_action = "ask_for_feedback"
                self.asked_feedback = True
                print("Ask for feedback: ", self.game_condition)
                return robot_action
        else:
            if not(self.condition_met):
                print("Unrecognized condition or already asked for feedback")
                self.condition_met = True
            return ""
        

if __name__ == '__main__':
    try:
        determine_action = DetermineAction()
        while not rospy.is_shutdown():
            determine_action.run()

    except rospy.ROSInterruptException:
        pass