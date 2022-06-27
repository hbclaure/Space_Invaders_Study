#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import qi
import time

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
        rospy.Subscriber('space_invaders/game/game_condition', String, self.game_condition_cb)
        rospy.Subscriber('space_invaders/game/game_mode', String, self.game_mode_cb)

        # Publishers 
        self.nao_action_pub = rospy.Publisher('space_invaders/game/nao_action',String,queue_size=5)

        # set up Nao
        self.nao_app = qi.Application(url="192.168.1.116")
        self.nao_app.start()
        self.tts_proxy = self.nao_app.session.service("ALTextToSpeech")
        self.motion_proxy = self.nao_app.session.service("ALMotion")
        self.posture_proxy = self.nao_app.session.service("ALRobotPosture")
        self.light_proxy = self.nao_app.session.service("ALLeds")
        
        # self.motion_proxy.moveInit()
        # starting position
        self.motion_proxy.wakeUp()
        self.posture_proxy.goToPosture("Sit",0.8)
        self.sleep_no_speech()

        self.tts_proxy.setVolume(1.5)
        self.said = False
    
    def game_mode_cb(self,msg):
        self.game_mode = int(msg.data)

    def nao_action_cb(self,msg):
        robot_action = msg.data
        if robot_action == "ask_for_feedback" and not(self.said):
            print("ask for feedback")
            self.ask_for_feedback()
            self.said = True
        elif robot_action == "introduction":
            self.introduction()
        elif robot_action == "sleep":
            self.sleep()
        elif robot_action == "wake":
            self.wake()
        elif robot_action == "":
            pass
        else:
            print("Unrecognized action: ", robot_action)

        #self.nao_action_pub.publish(nao_action)

    def game_condition_cb(self,msg):
        self.game_condition = msg.data
        self.said = False
        self.tts_proxy.setVolume(1.5)
        print("Game mode",self.game_mode)
        if self.game_mode >0:
            self.tts_proxy.say("\\style=default\\ Let's play!")
            #self.posture_proxy.goToPosture("Sit",0.8)  

    def ask_for_feedback(self):
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.3)
        #time.sleep(0.5)
        #self.wave()

        if self.game_condition in ['A','C']:
            self.tts_proxy.setVolume(1.5)
            self.tts_proxy.say("\\rspd=115\\ Please remember to press the up and down arrows to help \\emph=5\\ us \\pau=50\\ do better!")
        elif self.game_condition in ['B','D']:
            self.tts_proxy.setVolume(1.5)
            self.tts_proxy.say("\\rspd=115\\ Please remember to press the up and down arrows to help \\emph=5\\ me \\pau=50\\ do better!")
        else:
            print("Unrecognized condition")

        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.5)

    def wave(self):
        self.motion_proxy.wakeUp()
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
        angles = [.9, -.27, 1.26, .5, 0]
        times = [1.0, 1.0, 1.0, 1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, angles, times, True)
    
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
        angles = [-1.0, -1.2, 1.0, 0.5, 0]
        times = [0.5, 0.5, 1.0, 1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, angles, times, True)

        self.motion_proxy.openHand("RHand")
        for i in range(2):
            self.motion_proxy.setAngles("RElbowRoll", 1.5,0.5)
            time.sleep(0.25)
            self.motion_proxy.setAngles("RElbowRoll", 0.5,0.5)
            time.sleep(0.25)
        self.motion_proxy.setAngles("RElbowRoll", 1.0,0.5)
        self.motion_proxy.closeHand("RHand")
    
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
        angles = [.9, -.27, 1.26, .5, 0]
        times = [1.0, 1.0, 1.0, 1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, angles, times, True)
    
    def introduction(self):
        self.wake_movement_lights()
        self.tts_proxy.setVolume(1.5)
        self.tts_proxy.say("Hi I'm NAO! It's nice to meet you. I'm excited to play Space Invaders together!")
        self.wave()

    def sleep(self):
        self.tts_proxy.setVolume(1.5)
        self.tts_proxy.say("That was a good game. I'm going to sleep now!")
        self.sleep_no_speech()

    def sleep_no_speech(self):
        names = ["HeadYaw", "HeadPitch"]
        times = [1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, [0.0, 0.0], times, True)
        self.motion_proxy.angleInterpolation(names, [0.0, 1.0], times, True)
        self.light_proxy.off("FaceLeds")
        self.light_proxy.off("ChestLeds")
        self.motion_proxy.rest()

    def wake(self):
        self.wake_movement_lights()
        self.tts_proxy.setVolume(1.5)
        self.tts_proxy.say("Ready to play!")
    
    def wake_movement_lights(self):
        self.motion_proxy.wakeUp()
        names = ["HeadYaw", "HeadPitch"]
        times = [1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, [0.0, 0.0], times, True)
        self.light_proxy.on("FaceLeds")
        self.light_proxy.on("ChestLeds")

    def run(self):
        pass

if __name__ == '__main__':
    try:
        nao_action = NaoAction()
        while not rospy.is_shutdown():
            nao_action.run()

    except rospy.ROSInterruptException:
        pass