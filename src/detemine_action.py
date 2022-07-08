#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import json

WE_CONDITIONS = ['A','C']
I_CONDITIONS = ['B','D']

BEFORE_CONDITIONS = ['A','B']
AFTER_CONDITIONS = ['C','D']

GAME_ENEMIES = 9*4*3*2 # across * row per color * color * sides
MIDDLE = 600 # middle of x axis

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
        rospy.Subscriber('space_invaders/game/game_mode', String, self.game_mode_cb)

        # Publishers 
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)

        # Attributes
        self.game_condition = None
        self.asked_feedback = False
        self.said_strategy = False
        self.condition_met = False

    def game_mode_cb(self,msg):
        self.game_mode = int(msg.data)
    
    def determine_action_cb(self,game_state):
        try:
            if self.game_mode == 1:
                game_state_dict = json.loads(game_state.data)
                if 'enemies_left_positions' in game_state_dict.keys():
                    robot_action = self.action_from_game_state(game_state_dict)
                    self.robot_action_pub.publish(robot_action)
                    self.robot_action_pub.publish(robot_action)
            else:
                #game over
                pass
        except ValueError:
            print(game_state)


    def game_condition_cb(self,msg):
        self.game_condition = msg.data
        self.asked_feedback = False
        self.said_strategy = False
        self.condition_met = False

    def run(self):
        pass

    def action_from_game_state(self,game_state):
        enemies_left_positions = game_state['enemies_left_positions']
        enemies_right_positions = game_state['enemies_right_positions']
        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        total_enemies = num_left_enemies+num_right_enemies

        if not(self.said_strategy) and total_enemies < GAME_ENEMIES*.75 and game_state['ai_position']<MIDDLE*.75:
                robot_action = "highlight_change"
                print("HIGHLIGHT")
                print(self.said_strategy)
                self.said_strategy = True
                print(self.said_strategy)
                return robot_action

        if self.game_condition in BEFORE_CONDITIONS and not(self.asked_feedback):
            # asking for feedback before crossing to help
            if total_enemies < GAME_ENEMIES*.875:
                robot_action = "ask_for_feedback"
                self.asked_feedback = True
                print("Ask for feedback: ", self.game_condition)
                return robot_action
        elif self.game_condition in AFTER_CONDITIONS and not(self.asked_feedback): 
            # after for feedback after crossing to help
            if total_enemies <= GAME_ENEMIES*.65 and game_state["ai_position"] > MIDDLE*1.1:
                robot_action = "ask_for_feedback"
                self.asked_feedback = True
                print("Ask for feedback: ", self.game_condition)
                return robot_action
        else:
            return ""
        

if __name__ == '__main__':
    try:
        determine_action = DetermineAction()
        while not rospy.is_shutdown():
            determine_action.run()

    except rospy.ROSInterruptException:
        pass