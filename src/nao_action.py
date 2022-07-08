#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import qi
import time

WE_CONDITIONS = ['A','C']
I_CONDITIONS = ['B','D']

BEFORE_CONDITIONS = ['A','B']
AFTER_CONDITIONS = ['C','D']

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
        
        # starting position
        self.motion_proxy.wakeUp()
        self.posture_proxy.goToPosture("Sit",0.8)
        self.sleep_no_speech()

        self.tts_proxy.setVolume(1.5)
        

        # Attributes
        self.game_condition = None
        self.announced = False
        self.said = False
    
    def game_mode_cb(self,msg):
        self.game_mode = int(msg.data)

    def nao_action_cb(self,msg):
        robot_action = msg.data
        if robot_action == "ask_for_feedback" and not(self.said):
            self.ask_for_feedback()
            self.said = True
        elif robot_action == "highlight_change" and not(self.announced):
            self.highlight_new_behavior()
            self.announced = True
        elif robot_action == "introduction":
            self.introduction()
        elif robot_action == "game_over_nap":
            self.sleep()
        elif robot_action == "game_over_no_nap":
            self.no_nap()
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
        self.announced = False
        self.tts_proxy.setVolume(1.5)
        print("Game mode",self.game_mode)
        if self.game_mode > 0:
            self.tts_proxy.setVolume(1.5)
        if self.game_condition in WE_CONDITIONS:
            self.tts_proxy.say("We're ready to play!")
        elif self.game_condition in I_CONDITIONS:
            self.tts_proxy.say("I'm ready to play!")
        else:
            print("Unrecognized condition")
        #self.tts_proxy.say("\\style=default\\ Let's play!")
        #self.posture_proxy.goToPosture("Sit",0.8)  

    def ask_for_feedback(self):
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.3)
        self.tts_proxy.setVolume(1.5)

        if self.game_condition in WE_CONDITIONS:
            self.tts_proxy.say("\\rspd=115\\ Remember to give feedback so \\emph=5\\ we are \\pau=50\\ a better team!")
            print("a")
        elif self.game_condition in I_CONDITIONS:
            self.tts_proxy.say("\\rspd=115\\ Remember to give feedback so \\emph=5\\ I am \\pau=50\\ a better player!")
            print("b")
        else:
            print("Unrecognized condition")

        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.5)

    def highlight_new_behavior(self):
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.3)
        
        self.tts_proxy.setVolume(1.5)

        if self.game_condition in WE_CONDITIONS:
            print("um")
            self.tts_proxy.say("\\rspd=115\\ Look, \\emph=5\\ we are \\pau=50\\ destroying enemies on the left side of the screen!")
        elif self.game_condition in I_CONDITIONS:
            print("uh")
            self.tts_proxy.say("\\rspd=115\\ Look, \\emph=5\\ I am \\pau=50\\ destroying enemies on the left side of the screen!")
        else:
            print(f"Unknown condition: {self.game_condition}")
        
        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.5)

    def wave(self):
        self.motion_proxy.wakeUp()
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
        angles = [.9, -.27, 1.26, .5, 0]
        times = [1.0, 1.0, 1.0, 1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, angles, times, True)
    
        names = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RElbowYaw", "RWristYaw"]
        angles = [-0.25, -0.25, 1.0, 0.5, 0.75]
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
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.25)
        self.tts_proxy.setVolume(1.5)
        self.wave()
        self.tts_proxy.say("Hi I'm NAO! It's nice to meet you. I'm excited to play Space Invaders together!")
        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.25)

    def no_nap(self):
        if self.game_condition in WE_CONDITIONS:
            self.tts_proxy.say("We're done with that game!")
        elif self.game_condition in I_CONDITIONS:
            self.tts_proxy.say("I'm done with that game!")
        else:
            print("Unrecognized condition")
    
    def sleep(self):
        self.tts_proxy.setVolume(1.5)
        if self.game_condition in WE_CONDITIONS:
            self.tts_proxy.say("We're done with that game! I'm going to take a nap now while you answer some questions!")
        elif self.game_condition in I_CONDITIONS:
            self.tts_proxy.say("I'm done with that game! I'm going to take a nap now while you answer some questions!")
        else:
            print("Unrecognized condition")
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