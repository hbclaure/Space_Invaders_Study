#!/usr/bin/env python

# original imports
import logging
import tornado.escape
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket
import tornado.gen
import tornado.httpserver
import tornado.httputil
import os
import uuid
import json
import random
import ssl
from datetime import datetime
import sentry_sdk
from typing import Any

# ros imports
import rospy
from std_msgs.msg import String

# sentry info
sentry_sdk.init(
    "https://1d8e5b5288fa4616b900791c09abcfbb@o771330.ingest.sentry.io/5827220",

    # Set traces_sample_rate to 1.0 to capture 100%
    # of transactions for performance monitoring.
    # We recommend adjusting this value in production.
    traces_sample_rate=1.0
)

from game_files.agents.uncooperative import Uncooperative
from game_files.agents.cooperative_early import CooperativeEarly
from game_files.agents.cooperative_late import CooperativeLate
from game_files.agents.alternating_support import AlternatingSupport
from game_files.agents.sporadic_assistance import SporadicAssistance
from game_files.agents.targeted_assistance import TargetedAssistance

from game_files.agents.sporatic_early_fast import SporaticEarlyFast
from game_files.agents.sporatic_early_slow import SporaticEarlySlow
from game_files.agents.help_human_early import UnfairSupportHumanEarly
from game_files.agents.help_human_late import UnfairSupportHumanLate
from game_files.agents.help_shutter_early import UnfairSupportShutterEarly
from game_files.agents.help_shutter_late import UnfairSupportShutterLate

from game_files.agents.equal_support import EqualSupport
from game_files.agents.training import Training
from game_files.agents.robot_only_practice import ShutterPractice
from game_files.agents.nao_only_training import NaoTrain

from game_files.agents.apology import Apology

from tornado.options import define, options
define("port",default = 8667, help="run on the given port", type=int)
define("machine",default='anna',help="run on machine",type=str)

WEBROOT = os.path.dirname(os.path.realpath(__file__))
SSL_ROOT = "/home/si_app/ssl"

agents = {
    0: NaoTrain,
    1: Training,
    2: EqualSupport,
    3: UnfairSupportShutterLate,
    4: UnfairSupportShutterEarly,
    5: EqualSupport,
    6: Apology, 
    7: Apology
}
machines = {
    'anna': ("anna.cs.yale.edu.crt","anna.cs.yale.edu.key"),
    'xpm': ("apache.crt","apache.key")
}

class Application(tornado.web.Application):
    '''
    ROS node
    '''
    def __init__(self):
        # Initialize the node
        print("initializing websocket node")
        rospy.init_node('websocket')


        settings = dict(
            cookie_secret="a0ds8fhasldkgakl1joasdig0asdhgalkj30asgjdgla;ksd;glah",
            template_path=os.path.join(os.path.dirname(__file__), "templates"),
            static_path=os.path.join(os.path.dirname(__file__), "game_files/static"),
            xsrf_cookies=True,
        )
        handlers = [
            (r"/control", ControlHandler),
            (r"/image", ImageHandler),
            (r"/game", GameHandler),
            (r"/(.*)", tornado.web.StaticFileHandler, dict(path=settings['static_path'],default_filename="index.html"))
        ]
        super().__init__(handlers, **settings)

class ControlHandler(tornado.websocket.WebSocketHandler):
    # put in __init__
    def __init__(
        self,
        application: tornado.web.Application,
        request: tornado.httputil.HTTPServerRequest,
        **kwargs: Any
    ) -> None:
        super().__init__(application, request, **kwargs)
        self.player_id = None
        self.mode = None
        self.display_vid = None
        self.game_num = None
        self.time_label = None
        self.current_agent = None
        self.open_time = None
        self.logged = False
        self.control_msgs = 0
        self.dirname = "game_data"

        # Publisher
        self.game_state_pub = rospy.Publisher('space_invaders/game/game_state',String,queue_size=5)
        self.game_condition_pub = rospy.Publisher('space_invaders/game/game_condition',String,queue_size=5)
        self.game_mode_pub = rospy.Publisher('space_invaders/game/game_mode',String,queue_size=5)
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)
        self.frame_number_pub = rospy.Publisher('space_invaders/game/frame_number',String,queue_size=5)

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        self.open_time = datetime.utcnow()

    def on_close(self):
        #log player id, timestamp
        close_time = datetime.utcnow()
        path_to_socket = f"{self.dirname}/socket_logs/P{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}/control_P{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
        msg = {'player_id': self.player_id, 'mode': self.mode, 'game_num': self.game_num, 'display_vid': self.display_vid, 'open_time': self.open_time, 'close_time': close_time, 'logged': self.logged, 'control_msgs': self.control_msgs, 'code':self.close_code, 'reason':self.close_reason}
        if not os.path.exists(os.path.dirname(path_to_socket)):
            os.makedirs(os.path.dirname(path_to_socket))
        with open(path_to_socket,"w") as f:
            json.dump(msg,f,default=str)

    def on_message(self, msg):
        if not self.mode and self.mode != 0:
            try:
                state = json.loads(msg)
                print("these are the messages", state)
                self.player_id = state['player_id']
                self.mode = state['mode']
                if self.mode == 0:
                    self.dirname = "practice_game_data"
                print("Mode: ", self.mode)
                self.display_vid = state['display_vid']
                time = datetime.utcnow()
                self.time_label = f"{str(time.year)}_{str(time.month)}_{str(time.day)}_{str(time.hour)}_{str(time.minute)}"
                gn = state['game_num']
                while(os.path.exists(f"{self.dirname}/control_logs/ingame_P{self.player_id}_v{self.display_vid}_m{self.mode}_g{str(gn)}_t{self.time_label}.json")):
                    gn = int(gn)+100
                    print("increment Control")
                self.game_num = str(gn)
                print(f"game_num: {self.game_num}")
                  
                self.current_agent = agents[self.mode]()
                print("Current Agent: ", self.current_agent)
 

                # publish game mode and condition
                self.game_mode_pub.publish(str(state['mode']))
                self.game_condition_pub.publish(str(state['game_condition']))
            except Exception as e:
                print(f"{e} Mode or agent error")
                sentry_sdk.capture_exception(e)

        else:
            state = json.loads(msg)
            # publish game state
            state_string = json.dumps(state)
            self.game_state_pub.publish(state_string)
            self.control_msgs +=1

            if "frame_number" in state.keys():
                self.frame_number_pub.publish(str(state["frame_number"]))
                if self.mode == 0:
                    print('her')
                    # for practice mode, send empty action (do nothing)
                    #action = {'left': False, 'right': False, 'shoot': False,'support':1}
                    action = self.current_agent.update(state)
                    print('this is action', action)
                # elif self.mode == 4:
                #     action = {'left': False, 'right': False, 'shoot': False, 'support': 0}
                else:
                    action = self.current_agent.update(state)
                    #print('action',action)
                    
                path_to_control = f"{self.dirname}/control_logs/ingame_P{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
                if not os.path.exists(os.path.dirname(path_to_control)):
                    os.makedirs(os.path.dirname(path_to_control))
                with open(path_to_control,"a") as f:
                    f.write(msg)
                    f.write(",\n")
                    json.dump(action,f)
                    f.write(",\n")
                # send action
                self.write_message(json.dumps(action))
            else:
                print(f"events received: {self.player_id}")
                

                try:
                    if self.mode == 0:
                        p_stage = state['practice_stage']
                        path_to_json = f"{self.dirname}/game_logs/P{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}_stage{p_stage}.json"
                        if not os.path.exists(os.path.dirname(path_to_json)):
                            os.makedirs(os.path.dirname(path_to_json))
                        
                        with open(path_to_json,"w") as f:
                            f.write(msg)
                        print(f"STAGE: {p_stage}")
                        print(f"Saved stage {p_stage}/5 of practice")
                        if p_stage == 5:
                            self.logged = True
                            self.write_message("saved")
                    else:
                        if self.game_num[-1] in ['2','3','4','6']:
                            self.robot_action_pub.publish("game_over_nap")
                        else:
                            self.robot_action_pub.publish("game_over_no_nap")
                        path_to_json = f"participant_data/P{self.player_id}/{self.dirname}/game_logs/P{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
                        if not os.path.exists(os.path.dirname(path_to_json)):
                            os.makedirs(os.path.dirname(path_to_json))
                        
                        print(f"Starting to save: {self.player_id}")
                        with open(path_to_json,"w") as f:
                            f.write(msg)
                        self.logged = True
                        self.write_message("saved")
                        print(f"Gamelog Saved: {self.player_id}")
                except Exception as e:
                    print(f"Logging error: {self.player_id}")
                    print(e)
                    sentry_sdk.capture_exception(e)




class ImageHandler(tornado.websocket.WebSocketHandler):
    def __init__(
        self,
        application: tornado.web.Application,
        request: tornado.httputil.HTTPServerRequest,
        **kwargs: Any
    ) -> None:
        super().__init__(application, request, **kwargs)
        self.player_id = None
        self.mode = None
        self.game_num = None
        self.time_label = None
        self.start_frame_count = 0
        self.end_frame_count = 0
        self.display_vid = None
        self.open_time = None
        self.stage = []    
        self.dirname = "game_data"
        self.type="webcam"
        self.folder = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        self.open_time = datetime.utcnow()

    def on_close(self):
        #log player id, timestamp
        close_time = datetime.utcnow()
        path_to_socket = f"participant_data/P{self.player_id}/{self.dirname}/socket_logs/{self.folder}/{self.type}_{self.folder}.json"
        msg = {'player_id': self.player_id, 'mode': self.mode, 'game_num': self.game_num, 'display_vid': self.display_vid, 'open_time': self.open_time, 'close_time': close_time, 'stage': self.stage, 'code':self.close_code, 'reason':self.close_reason}
        if not os.path.exists(os.path.dirname(path_to_socket)):
            os.makedirs(os.path.dirname(path_to_socket))
        with open(path_to_socket,"w") as f:
            json.dump(msg,f,default=str)

    def on_message(self, msg):
        if not self.player_id:
            try:
                state = json.loads(msg)
                self.player_id = state['player_id']
                self.mode = state['mode']
                if self.mode == 0:
                    self.dirname = "practice_game_data"
                self.display_vid = state['display_vid']
                time = datetime.utcnow()
                self.time_label = f"{str(time.year)}_{str(time.month)}_{str(time.day)}_{str(time.hour)}_{str(time.minute)}"
                gn = state['game_num']
                while(os.path.exists(os.path.dirname(f"participant_data/P{self.player_id}/{self.dirname}/recorded_frames/P{str(self.player_id)}_v{str(self.display_vid)}_m{str(self.mode)}_g{str(gn)}_t{str(self.time_label)}/{self.type}_start/"))):
                    gn = int(gn)+100
                    print(f"increment {self.type}")
                self.game_num = str(gn)
                self.folder = f"P{str(self.player_id)}_v{str(self.display_vid)}_m{str(self.mode)}_g{str(self.game_num)}_t{str(self.time_label)}"

                # make directories
                folders = [f'participant_data/P{self.player_id}/{self.dirname}/recorded_frames/', self.logging_path(), self.start_path(0,0), self.in_game_path(0,0,0), self.end_path(0,0)]
                for folder in folders:
                    if not os.path.exists(os.path.dirname(folder)):
                            os.makedirs(os.path.dirname(folder))
                
                with open(self.logging_path(), "+a") as f:
                    f.write(f'Log file created\n')

                sentry_sdk.set_context("user", {
                    "id": self.player_id,
                    "mode": self.mode,
                    "time": time
                })
                
                print(f"Player: {self.player_id}")
            except Exception as e:
                print(e)
                print("invalid - no player_id")
                sentry_sdk.capture_exception(e)
        else:
            try:
                if str(msg)[0:5] == 'EVENT':
                    with open(self.logging_path(), "+a") as f:
                        f.write(msg + '\n')
                else:
                    first_z = str(msg).find("z")
                    first_y = str(msg).find("y")
                    first_w = str(msg).find("w")
                    first_backslash = str(msg).find("\\")
                    frame_number =str(msg)[2:first_z]
                    frame_number = int(frame_number)
                    stage = str(msg)[first_z+1:first_y]
                    stage = int(stage)
                    millis = str(msg)[first_y+1:first_w]
                    millis = int(millis)
                    current_millis = str(msg)[first_w+1:first_backslash]
                    current_millis = int(current_millis)
                    image = msg[first_backslash-2:]
                    if image:
                        if stage == 1:
                            if not(1 in self.stage):
                                self.stage.append(1)
                            filename = self.in_game_path(frame_number,millis,current_millis)
                        elif stage == 0:
                            if not(0 in self.stage):
                                self.stage.append(0)
                            filename = self.start_path(millis,current_millis)
                        elif stage == 2:
                            if not(2 in self.stage):
                                self.stage.append(2)
                            filename = self.end_path(millis,current_millis)
                        # if not os.path.exists(os.path.dirname(filename)):
                        #     os.makedirs(os.path.dirname(filename))
                        with open(filename, "+wb") as f:
                            f.write(image)
                        # if os.path.exists(filename):
                        #     with open(self.logging_path(), "+a") as f:
                        #         f.write(f'Image Already Saved: s{self.stage[-1]}, f{frame_number}, m{millis}\n')
                        # with open(self.logging_path(), "+a") as f:
                        #     f.write(f'Image Saved: s{self.stage[-1]}, f{frame_number}, m{millis}\n')
                    else:
                        with open(self.logging_path(), "+a") as f:
                            f.write("NO_IMAGE_FOUND: " + str(msg) + '\n')
            except Exception as e:
                print(e)
                print(f"error saving images: {self.player_id}")
                if not os.path.exists(os.path.dirname(self.logging_path())):
                            os.makedirs(os.path.dirname(self.logging_path()))
                with open(self.logging_path(), "+a") as f:
                    f.write(f"error saving images: {self.player_id} at {datetime.utcnow()}\n, {e}\n")
                sentry_sdk.capture_exception(e)

    def start_path(self,millis,current_millis):
        self.start_frame_count += 1
        # folder = f"P{str(self.player_id)}_v{str(self.display_vid)}_m{str(self.mode)}_g{str(self.game_num)}_t{str(self.time_label)}"
        filename = f"participant_data/P{self.player_id}/{self.dirname}/recorded_frames/{self.folder}/{self.type}_start/start_{self.start_frame_count:05d}_m{millis}_cm{current_millis}.jpg"
        return filename

    def in_game_path(self, frame_number,millis,current_millis):
        # folder = f"P{str(self.player_id)}_v{str(self.display_vid)}_m{str(self.mode)}_g{str(self.game_num)}_t{str(self.time_label)}"
        filename = f"participant_data/P{self.player_id}/{self.dirname}/recorded_frames/{self.folder}/{self.type}/w_{frame_number:05d}_m{millis}_cm{current_millis}.jpg"
        return filename

    def end_path(self,millis,current_millis):
        self.end_frame_count += 1
        # folder = f"P{str(self.player_id)}_v{str(self.display_vid)}_m{str(self.mode)}_g{str(self.game_num)}_t{str(self.time_label)}"
        filename = f"participant_data/P{self.player_id}/{self.dirname}/recorded_frames/{self.folder}/{self.type}_end/end_{self.end_frame_count:05d}_m{millis}_cm{current_millis}.jpg"
        return filename
    
    def logging_path(self):
        return f"participant_data/P{self.player_id}/{self.dirname}/{self.type}_log/{self.folder}.log"

class GameHandler(ImageHandler):
    def __init__(self, *args, **kwargs: Any):
        super().__init__(*args, **kwargs)
        self.type = "gamescreen"

def main():
    app = Application()
    tornado.options.parse_command_line()
    port = options.port
    machine = options.machine
    proto = 'http'
    ssl_options = {}
    # if os.path.exists(SSL_ROOT):
    #     crt = machines[machine][0]
    #     key = machines[machine][1]
    #     ssl_options['certfile'] = os.path.join(SSL_ROOT, crt)
    #     ssl_options['keyfile'] = os.path.join(SSL_ROOT, key)
    #     proto = 'https'
    #     app.listen(port, '0.0.0.0', ssl_options=ssl_options)
    # else:
    
    try:
        app.listen(port, '127.0.0.1')
        print(f"Listening on {proto}://127.0.0.1:%i" % port)
        while not rospy.is_shutdown():
            print("starting loop")   
            tornado.ioloop.IOLoop.current().start()
    except KeyboardInterrupt:
        print("key")
        pass
    except rospy.ROSInterruptException:
        print("ros")
        pass
    finally:
        tornado.ioloop.IOLoop.current().stop()
        tornado.ioloop.IOLoop.current().close(True)
    

if __name__ == "__main__":
    main()

