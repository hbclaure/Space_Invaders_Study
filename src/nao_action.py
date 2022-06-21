#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import qi


    

class NaoAction():
    '''
    ROS node
    '''
    def __init__(self):
        # Initialize the node
        print("initializing nao action node")
        rospy.init_node('determine_nao_action')
        
        # Subscribers
        rospy.Subscriber('space_invaders/game/robot_action', String, self.nao_action_cb)

        # Publishers 
        self.nao_action_pub = rospy.Publisher('space_invaders/game/nao_action',String,queue_size=5)

        # set up Nao
        self.nao_app = qi.Application(url="192.168.1.104")
        self.nao_app.start()
        self.tts_proxy = self.nao_app.session.service("ALTextToSpeech")
        self.motion_proxy = self.nao_app.session.service("ALMotion")

        self.said = False

    def nao_action_cb(self,msg):
        robot_action = msg.data
        print(robot_action)
        if robot_action == "hi" and not(self.said):
            print("hmmmm")
            self.tts_proxy.say("on the left")
            self.said = True
        elif robot_action == "right":
            print("reset")
            self.said = False

        #self.nao_action_pub.publish(nao_action)


    
    def run(self):
        pass

if __name__ == '__main__':
    try:
        nao_action = NaoAction()
        while not rospy.is_shutdown():
            nao_action.run()

    except rospy.ROSInterruptException:
        pass