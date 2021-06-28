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
import sqlite3
import random
import ssl
from datetime import datetime
import base64
import sentry_sdk
from typing import Any

sentry_sdk.init(
    "https://1d8e5b5288fa4616b900791c09abcfbb@o771330.ingest.sentry.io/5827220",

    # Set traces_sample_rate to 1.0 to capture 100%
    # of transactions for performance monitoring.
    # We recommend adjusting this value in production.
    traces_sample_rate=1.0
)

from agents.uncooperative import Uncooperative
from agents.cooperative_early import CooperativeEarly
from agents.cooperative_late import CooperativeLate

from tornado.options import define, options
define("port",default = 9999, help="run on the given port", type=int)
define("machine",default='anna',help="run on machine",type=str)

WEBROOT = os.path.dirname(os.path.realpath(__file__))
DATABASE = os.path.join(WEBROOT, 'db/game_logs.db')
#SSL_ROOT = "/etc/apache2/ssl"

agents = {
    1: CooperativeEarly,
    2: CooperativeLate,
    3: Uncooperative
}

#machines = {
#    'anna': ("anna.cs.yale.edu.crt","anna.cs.yale.edu.key"),
#    'xpm': ("apache.crt","apache.key")
#}

class Application(tornado.web.Application):
    def __init__(self):
        settings = dict(
            cookie_secret="a0ds8fhasldkgakl1joasdig0asdhgalkj30asgjdgla;ksd;glah",
            template_path=os.path.join(os.path.dirname(__file__), "templates"),
            static_path=os.path.join(os.path.dirname(__file__), "static"),
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

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        self.open_time = datetime.utcnow()

    def on_close(self):
        #log player id, timestamp
        close_time = datetime.utcnow()
        path_to_socket = f"socket_logs/{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}/control_{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
        msg = {'player_id': self.player_id, 'mode': self.mode, 'game_num': self.game_num, 'display_vid': self.display_vid, 'open_time': self.open_time, 'close_time': close_time, 'logged': self.logged, 'control_msgs': self.control_msgs, 'code':self.close_code, 'reason':self.close_reason}
        if not os.path.exists(os.path.dirname(path_to_socket)):
            os.makedirs(os.path.dirname(path_to_socket))
        with open(path_to_socket,"w") as f:
            json.dump(msg,f,default=str)

    def on_message(self, msg):
        if not self.mode:
            try:
                state = json.loads(msg)
                self.player_id = state['player_id']
                self.mode = state['mode']
                print("Mode: ", self.mode)
                self.display_vid = state['display_vid']
                time = datetime.utcnow()
                self.time_label = str(time.year)+"_" + str(time.month)+"_" + str(time.day)+"_" + str(time.hour)+"_" + str(time.minute)
                self.game_num = state['game_num']
                
                self.current_agent = agents[self.mode]()
                print("Current Agent: ", self.current_agent)
            except Exception as e:
                print(f"{e} Mode or agent error")
                sentry_sdk.capture_exception(e)

        else:
            state = json.loads(msg)
            self.control_msgs +=1

            if "frame_number" in state.keys():
                action = self.current_agent.update(state)
                
                path_to_control = f"control_logs/ingame_{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
                if not os.path.exists(os.path.dirname(path_to_control)):
                    os.makedirs(os.path.dirname(path_to_control))
                with open(path_to_control,"a") as f:
                    f.write(msg)
                    f.write("\n")
                    json.dump(action,f)
                    f.write("\n")

                # send action
                self.write_message(json.dumps(action))
            else:
                print(f"events received: {self.player_id}")

                try:
                    path_to_json = f"game_logs/{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
                    with open(path_to_json,"w") as f:
                        f.write(msg)
                    self.logged = True
                    self.write_message("saved")
                    print(f"Saved to database: {self.player_id}")
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

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        self.open_time = datetime.utcnow()

    def on_close(self):
        #log player id, timestamp
        close_time = datetime.utcnow()
        path_to_socket = f"socket_logs/{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}/image_{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
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
                self.display_vid = state['display_vid']
                time = datetime.utcnow()
                self.time_label = str(time.year)+"_" + str(time.month)+"_" + str(time.day)+"_" + str(time.hour)+"_" + str(time.minute)
                gn = state['game_num']
                while(os.path.exists(os.path.dirname(f"recorded_frames/P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(gn)+"_t"+str(self.time_label)+"/"))):
                    gn = int(gn)+100
                self.game_num = str(gn)
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
                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))
                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(e)
                print(f"error saving images: {self.player_id}")
                sentry_sdk.capture_exception(e)

    def start_path(self,millis,current_millis):
        self.start_frame_count += 1
        folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
        filename = f"recorded_frames/{folder}/webcam_start/start_{self.start_frame_count:05d}_m{millis}_cm{current_millis}.jpg"
        return filename

    def in_game_path(self, frame_number,millis,current_millis):
        folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
        filename = f"recorded_frames/{folder}/webcam/w_{frame_number:05d}_m{millis}_cm{current_millis}.jpg"
        return filename

    def end_path(self,millis,current_millis):
        self.end_frame_count += 1
        folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
        filename = f"recorded_frames/{folder}/webcam_end/end_{self.end_frame_count:05d}_m{millis}_cm{current_millis}.jpg"
        return filename

class GameHandler(tornado.websocket.WebSocketHandler):
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
        self.display_vid = None
        self.open_time = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        self.open_time = datetime.utcnow()

    def on_close(self):
        #log player id, timestamp
        close_time = datetime.utcnow()
        path_to_socket = f"socket_logs/{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}/game_{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
        msg = {'player_id': self.player_id, 'mode': self.mode, 'game_num': self.game_num, 'display_vid': self.display_vid, 'open_time': self.open_time, 'close_time': close_time, 'code':self.close_code, 'reason':self.close_reason}
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
                self.display_vid = state['display_vid']
                time = datetime.utcnow()
                self.time_label = str(time.year)+"_" + str(time.month)+"_" + str(time.day)+"_" + str(time.hour)+"_" + str(time.minute)
                gn = state['game_num']
                while(os.path.exists(os.path.dirname(f"recorded_frames/P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(gn)+"_t"+str(self.time_label)+"/"))):
                    gn = int(gn)+100
                self.game_num = str(gn)
                print(f"Recording frames: {self.player_id}")
            except Exception as e:
                print(e)
                print("invalid - no player_id")
                sentry_sdk.capture_exception(e)

        else:
            try:
                first_z = str(msg).find("z")
                first_w = str(msg).find("w")
                first_backslash = str(msg).find("\\")
                frame_number =str(msg)[2:first_z]
                frame_number = int(frame_number)
                millis = str(msg)[first_z+1:first_w]
                millis = int(millis)
                current_millis = str(msg)[first_w+1:first_backslash]
                current_millis = int(current_millis)
                image = msg[first_backslash-2:]
                if image:
                    folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
                    filename = f"recorded_frames/{folder}/gamescreen/g_{frame_number:05d}_m{millis}_cm{current_millis}.jpg"
                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))
                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(f"{e} {self.player_id}")
                print(f"error saving frames: {self.player_id}")
                sentry_sdk.capture_exception(e)


def main():
    app = Application()
    tornado.options.parse_command_line()
    port = options.port
    machine = options.machine
    proto = 'http'
    #ssl_options = {}
    #if os.path.exists(SSL_ROOT):
    #    crt = machines[machine][0]
    #    key = machines[machine][1]
    #    ssl_options['certfile'] = os.path.join(SSL_ROOT, crt)
    #    ssl_options['keyfile'] = os.path.join(SSL_ROOT, key)
    #    proto = 'https'
    #    app.listen(port, '0.0.0.0', ssl_options=ssl_options)
    #else:
    app.listen(port, '127.0.0.1')
    print(f"Listening on {proto}://127.0.0.1:%i" % port)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()

