#!/usr/bin/env python

# ros imports
import rospy
from std_msgs.msg import String

# other imports
import qi
import time
import random
import json

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
        rospy.Subscriber('space_invaders/game/frame_number', String, self.set_frame_cb)

        # Publishers 
        self.nao_action_pub = rospy.Publisher('space_invaders/game/nao_action',String,queue_size=5)

        # set up Nao
        self.nao_app = qi.Application(url="192.168.1.116")
        self.nao_app.start()
        self.tts_proxy = self.nao_app.session.service("ALTextToSpeech")
        self.motion_proxy = self.nao_app.session.service("ALMotion")
        self.posture_proxy = self.nao_app.session.service("ALRobotPosture")
        self.light_proxy = self.nao_app.session.service("ALLeds")
        

        # Attributes
        self.game_condition = None
        self.game_mode = 0
        self.announced = False
        self.said = False
        self.last_time = time.time()
        self.next_time = random.randint(0,10)
        self.next_angle = random.randint(0,3)
        self.meaningful_action = False
        self.game_over = False
        self.frame_num = 0

         # starting position
        self.motion_proxy.wakeUp()
        self.posture_proxy.goToPosture("Sit",0.8)
        self.nao_action_pub.publish(f"frame_{self.frame_num} sit")
        self.sleep_no_speech()

        self.tts_proxy.setVolume(1.5)
    
    def game_mode_cb(self,msg):
        self.game_mode = int(msg.data)
        self.frame_num = 0

    def set_frame_cb(self,msg):
        self.frame_num = int(msg.data)

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
        self.game_over = False
        if self.game_mode > 0:
            self.tts_proxy.setVolume(1.5)
            if self.game_condition in WE_CONDITIONS:
                self.nao_action_pub.publish(f"frame_{self.frame_num} we_ready")
                self.tts_proxy.say("We're ready to play!")
            elif self.game_condition in I_CONDITIONS:
                self.nao_action_pub.publish(f"frame_{self.frame_num} i_play")
                self.tts_proxy.say("I'm ready to play!")
            else:
                print("Unrecognized condition")
        #self.tts_proxy.say("\\style=default\\ Let's play!")
        #self.posture_proxy.goToPosture("Sit",0.8)  

    def ask_for_feedback(self):
        if self.game_condition in BEFORE_CONDITIONS:
            self.meaningful_action = True
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.3)
        self.nao_action_pub.publish(f"frame_{self.frame_num} ask_look")
        self.tts_proxy.setVolume(1.5)

        if self.game_condition in WE_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} ask_we")
            self.tts_proxy.say("\\rspd=115\\ Remember to give feedback so \\emph=5\\ we are \\pau=50\\ a better team!")
            print("a")
        elif self.game_condition in I_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} ask_i")
            self.tts_proxy.say("\\rspd=115\\ Remember to give feedback so \\emph=5\\ I am \\pau=50\\ a better player!")
            print("b")
        else:
            print("Unrecognized condition")

        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.5)
        self.nao_action_pub.publish(f"frame_{self.frame_num} ask_return")
        if self.game_condition in AFTER_CONDITIONS:
            self.meaningful_action = False

    def highlight_new_behavior(self):
        if self.game_condition in AFTER_CONDITIONS:
            self.meaningful_action = True
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.3)
        self.nao_action_pub.publish(f"frame_{self.frame_num} highlight_look")
        
        self.tts_proxy.setVolume(1.5)

        if self.game_condition in WE_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} highlight_we")
            self.tts_proxy.say("\\rspd=115\\ Look, \\emph=5\\ we are \\pau=50\\ destroying enemies on the left side of the screen!")
        elif self.game_condition in I_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} highlight_i")
            self.tts_proxy.say("\\rspd=115\\ Look, \\emph=5\\ I am \\pau=50\\ destroying enemies on the left side of the screen!")
        else:
            print(f"Unknown condition: {self.game_condition}")
        
        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.5)
        self.nao_action_pub.publish(f"frame_{self.frame_num} highlight_return")
        if self.game_condition in BEFORE_CONDITIONS:
            self.meaningful_action = False

    def wave(self):
        self.nao_action_pub.publish(f"frame_{self.frame_num} wave_start")
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
        self.nao_action_pub.publish(f"frame_{self.frame_num} wave_stop")
    
    def introduction(self):
        self.wake_movement_lights()
        self.nao_action_pub.publish(f"frame_{self.frame_num} introduction_start")
        self.motion_proxy.setAngles("HeadYaw", 1.0,0.25)
        self.tts_proxy.setVolume(1.5)
        self.wave()
        self.tts_proxy.say("Hi I'm NAO! It's nice to meet you. I'm excited to play Space Invaders together!")
        time.sleep(0.5)
        self.motion_proxy.setAngles("HeadYaw", 0.0,0.25)
        self.nao_action_pub.publish(f"frame_{self.frame_num} introduction_end")

    def no_nap(self):
        self.game_over = True
        if self.game_condition in WE_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} no_nap_we")
            self.tts_proxy.say("We're done with that game!")
        elif self.game_condition in I_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} no_nap_i")
            self.tts_proxy.say("I'm done with that game!")
        else:
            print("Unrecognized condition")
    
    def sleep(self):
        self.game_over = True
        self.tts_proxy.setVolume(1.5)
        if self.game_condition in WE_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} sleep_we")
            self.tts_proxy.say("We're done with that game! I'm going to take a nap now while you answer some questions!")
        elif self.game_condition in I_CONDITIONS:
            self.nao_action_pub.publish(f"frame_{self.frame_num} sleep_i")
            self.tts_proxy.say("I'm done with that game! I'm going to take a nap now while you answer some questions!")
        else:
            print("Unrecognized condition")
        self.sleep_no_speech()

    def sleep_no_speech(self):
        self.nao_action_pub.publish(f"frame_{self.frame_num} sleep_off")
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
        self.nao_action_pub.publish(f"frame_{self.frame_num} wake_start")
        self.motion_proxy.wakeUp()
        names = ["HeadYaw", "HeadPitch"]
        times = [1.0, 1.0]
        self.motion_proxy.angleInterpolation(names, [0.0, 0.0], times, True)
        self.light_proxy.on("FaceLeds")
        self.light_proxy.on("ChestLeds")
        self.nao_action_pub.publish(f"frame_{self.frame_num} wake_end")

    def run(self):
        #print(not(self.meaningful_action), not(self.game_over), self.game_mode >0)

        if time.time()-self.last_time > self.next_time and not(self.meaningful_action) and not(self.game_over) and self.game_mode >0:
            names = ["HeadYaw","HeadPitch"]
            times = [.5,.5]
            angles = [[.1,.1],[-.1,.1],[.1,-.1],[-.1,-.1]]
            this_angle = self.next_angle
            self.nao_action_pub.publish(f"frame_{self.frame_num} idle_{this_angle}")
            self.motion_proxy.angleInterpolation(names,angles[this_angle],times,True)
            self.next_time = random.randint(8,15)
            while self.next_angle == this_angle:
                self.next_angle = random.randint(0,3)
            self.last_time = time.time()


if __name__ == '__main__':
    try:
        nao_action = NaoAction()
        while not rospy.is_shutdown():
            nao_action.run()

    except rospy.ROSInterruptException:
        pass