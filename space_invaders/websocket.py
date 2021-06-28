import logging
import tornado.escape
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket
import tornado.gen
import tornado.httpserver
import os
import uuid
import json
import sqlite3
import random
import ssl
from datetime import datetime
import base64
import sentry_sdk

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
from agents.apology import Apology

from tornado.options import define, options
define("port",default = 8668, help="run on the given port", type=int)
define("machine",default='anna',help="run on machine",type=str)

WEBROOT = os.path.dirname(os.path.realpath(__file__))
DATABASE = os.path.join(WEBROOT, 'db/game_logs.db')
SSL_ROOT = "/home/si_app/ssl"

agents = {
    1: CooperativeEarly,
    2: CooperativeLate,
    3: Uncooperative,
    4: Apology,
    5: Apology,
    6: Apology, 
    7: Apology
}

machines = {
    'anna': ("anna.cs.yale.edu.crt","anna.cs.yale.edu.key"),
    'xpm': ("apache.crt","apache.key")
}

class Application(tornado.web.Application):
    def __init__(self):
        settings = dict(
            cookie_secret="a0ds8fhasldkgakl1joasdig0asdhgalkj30asgjdgla;ksd;glah",
            template_path=os.path.join(os.path.dirname(__file__), "templates"),
            static_path=os.path.join(os.path.dirname(__file__), "static"),
            xsrf_cookies=True,
        )
        handlers = [
            #(r"/log", LogHandler),
            (r"/control", ControlHandler),
            (r"/image", ImageHandler),
            (r"/game", GameHandler),
            (r"/(.*)", tornado.web.StaticFileHandler, dict(path=settings['static_path'],default_filename="index.html"))
        ]
        super().__init__(handlers, **settings)

class ControlHandler(tornado.websocket.WebSocketHandler):
    mode = None
    current_agent = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True

    def on_message(self, msg):
        timestamp = datetime.utcnow()

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

            if "frame_number" in state.keys():
                action = self.current_agent.update(state)

                #state['action'] = action
                #state['timestamp'] = timestamp
                #self.logs.append(state)

                # send action
                self.write_message(json.dumps(action))
            else:
                # log = json.loads(msg)
                # self.frames = log['frames']
                # self.events = log['events']
                # self.player_id = log['player_id']
                # self.date = log['date']
                print(f"events received: {self.player_id}")

                try:
                    path_to_json = f"game_logs/{self.player_id}_v{self.display_vid}_m{self.mode}_g{self.game_num}_t{self.time_label}.json"
                    with open(path_to_json,"w") as f:
                        f.write(msg)
                    #self.log_data()
                    self.write_message("saved")
                    print(f"Saved to database: {self.player_id}")
                except Exception as e:
                    print(f"Logging error: {self.player_id}")
                    print(e)
                    sentry_sdk.capture_exception(e)




class ImageHandler(tornado.websocket.WebSocketHandler):

    player_id = None
    mode = None
    game_num = None
    time_label = None
    start_frame_count = 0
    end_frame_count = 0
    display_vid = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        pass

    def on_close(self):
        pass

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
                #r_msg = json.loads(msg)
                #if 'frame_number' in r_msg.keys():   
                #frame_number = r_msg['frame_number']
                #stage = r_msg['stage']
                #millis = r_msg['millis']
                #image = base64.b64decode(r_msg['img'].split('base64')[-1])
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
                #frame_number = r_msg['frame_number']
                #image = base64.b64decode(r_msg['img'].split('base64')[-1])
                image = msg[first_backslash-2:]
                if image:
                    if stage == 1:
                        filename = self.in_game_path(frame_number,millis,current_millis)
                    elif stage == 0:
                        filename = self.start_path(millis,current_millis)
                    elif stage == 2:
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
    
    player_id = None
    mode = None
    game_num = None
    time_label = None
    display_vid = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        pass

    def on_close(self):
        pass

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
                #r_msg = json.loads(msg)
                #if 'frame_number' in r_msg.keys():
                first_z = str(msg).find("z")
                first_w = str(msg).find("w")
                first_backslash = str(msg).find("\\")
                frame_number =str(msg)[2:first_z]
                frame_number = int(frame_number)
                millis = str(msg)[first_z+1:first_w]
                millis = int(millis)
                current_millis = str(msg)[first_w+1:first_backslash]
                current_millis = int(current_millis)
                #frame_number = r_msg['frame_number']
                #image = base64.b64decode(r_msg['img'].split('base64')[-1])
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
    ssl_options = {}
    if os.path.exists(SSL_ROOT):
        crt = machines[machine][0]
        key = machines[machine][1]
        ssl_options['certfile'] = os.path.join(SSL_ROOT, crt)
        ssl_options['keyfile'] = os.path.join(SSL_ROOT, key)
        proto = 'https'
        app.listen(port, '0.0.0.0', ssl_options=ssl_options)
    else:
        app.listen(port, '127.0.0.1')
    print(f"Listening on {proto}://127.0.0.1:%i" % port)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()

