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
    #logs = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def log_data(self):
        con = sqlite3.connect(DATABASE)
        cur = con.cursor()

        current_event = 0

        cur.execute('INSERT INTO Games(player_id, date, mode) VALUES(?,?,?)',
                                   (self.player_id, self.date, self.mode))

        game_id = cur.lastrowid

        for frame in self.frames:
            # log the actual frame and collect the id
            cur.execute('''INSERT INTO Frames(game_id, frame_number, timestamp, player_position, player_lives,
                        player_score, ai_position, ai_lives, ai_score, frame_sent) VALUES(?,?,?,?,?,?,?,?,?,?)''',
                            (game_id, frame['frame_number'], frame['timestamp'], frame['player_position'], frame['player_lives'],
                            frame['player_score'], frame['ai_position'], frame['ai_lives'], frame['ai_score'], frame['frame_sent']))
            frame_id = cur.lastrowid;

            # log the player bullet if it exists
            for bullet in frame['player_bullets_positions']:
                cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Player\',?,?,?)',
                                (frame_id, bullet[0], bullet[1]))

            # log the ai bullet if it exists
            for bullet in frame['ai_bullets_positions']:
                cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'AI\',?,?,?)',
                                (frame_id, bullet[0], bullet[1]))

            # log every enemy on the left
            for enemy in frame['enemies_left_positions']:
                cur.execute('INSERT INTO Enemies(side, frame_id, x, y) VALUES(\'Left\',?,?,?)',
                            (frame_id, enemy[0], enemy[1]))

            # log every enemy bullet on the left
            for bullet in frame['bullets_left_positions']:
                cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Left\',?,?,?)',
                                (frame_id, bullet[0], bullet[1]))

            # log every enemy on the right
            for enemy in frame['enemies_right_positions']:
                cur.execute('INSERT INTO Enemies(side, frame_id, x, y) VALUES(\'Right\',?,?,?)',
                            (frame_id, enemy[0], enemy[1]))

            # log every enemy bullet on the right
            for bullet in frame['bullets_right_positions']:
                cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Right\',?,?,?)',
                                (frame_id, bullet[0], bullet[1]))
            
            # log any events that occurred in this frame
            while (current_event < len(self.events) and self.events[current_event]['frame'] == frame['frame_number']):
                cur.execute('INSERT INTO Events(frame_id, killer, killed) VALUES(?,?,?)',
                            (frame_id, self.events[current_event]['killer'], self.events[current_event]['killed']))
                current_event += 1

            try:
                cur.execute('INSERT INTO Actions(game_id, frame_id, frame_sent, player_left, player_right, player_shoot, player_tried, player_signal_down, player_signal_up, player_tried_down, player_tried_up, ai_actual_left, ai_actual_right, ai_actual_shoot, ai_rec_left, ai_rec_right, ai_rec_shoot) VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                            (game_id, frame_id, frame['frame_sent'],
                             frame['player_action']['left'], frame['player_action']['right'], frame['player_action']['shoot'], frame['player_action']['tried_to_shoot'],
                             frame['signal_down'], frame['signal_up'], frame['tried_signal_down'], frame['tried_signal_up'],
                             frame['ai_actual_action']['left'], frame['ai_actual_action']['right'], frame['ai_actual_action']['shoot'],
                             frame['ai_received_action']['left'], frame['ai_received_action']['right'], frame['ai_received_action']['shoot']))
            except Exception as e:
                print("Action error: ", frame['frame_number'])
                print(e)
                
        con.commit()
        con.close()
        # self.write_message('saved')

    def on_message(self, msg):
        timestamp = datetime.utcnow()

        if not self.mode:
            try:
                self.mode = int(json.loads(msg))
                print("Mode: ", self.mode)
                self.current_agent = agents[self.mode]()
                print("Current Agent: ", self.current_agent)
                self.logs = []
            except Exception as e:
                print(e)
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
                log = json.loads(msg)
                self.frames = log['frames']
                self.events = log['events']
                self.player_id = log['player_id']
                self.date = log['date']
                print("events received")

                try:
                    self.log_data()
                    self.write_message("saved")
                    print(f"Saved to database: {self.player_id}")
                except Exception as e:
                    print("Logging error")
                    print(e)



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
                
                print(f"Player: {self.player_id}")
            except Exception as e:
                print(e)
                print("invalid")
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
                first_backslash = str(msg).find("\\")
                frame_number =str(msg)[2:first_z]
                frame_number = int(frame_number)
                stage = str(msg)[first_z+1:first_y]
                stage = int(stage)
                millis = str(msg)[first_y+1:first_backslash]
                millis = int(millis)
                #frame_number = r_msg['frame_number']
                #image = base64.b64decode(r_msg['img'].split('base64')[-1])
                image = msg[first_backslash-2:]
                if image:
                    if stage == 1:
                        filename = self.in_game_path(frame_number,millis)
                    elif stage == 0:
                        filename = self.start_path(millis)
                    elif stage == 2:
                        filename = self.end_path(millis)
                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))
                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(e)
                print("error")

    def start_path(self,millis):
        self.start_frame_count += 1
        folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
        filename = f"recorded_frames/{folder}/webcam_start/start_{self.start_frame_count:05d}_m{millis}.jpg"
        return filename

    def in_game_path(self, frame_number,millis):
        folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
        filename = f"recorded_frames/{folder}/webcam/w_{frame_number:05d}_m{millis}.jpg"
        return filename

    def end_path(self,millis):
        self.end_frame_count += 1
        folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
        filename = f"recorded_frames/{folder}/webcam_end/end_{self.end_frame_count:05d}_m{millis}.jpg"
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
                print("invalid")
        else:
            try:
                #r_msg = json.loads(msg)
                #if 'frame_number' in r_msg.keys():
                first_z = str(msg).find("z")
                first_backslash = str(msg).find("\\")
                frame_number =str(msg)[2:first_z]
                frame_number = int(frame_number)
                millis = str(msg)[first_z+1:first_backslash]
                millis = int(millis)
                #frame_number = r_msg['frame_number']
                #image = base64.b64decode(r_msg['img'].split('base64')[-1])
                image = msg[first_backslash-2:]
                if image:
                    folder = "P"+str(self.player_id)+"_v"+str(self.display_vid)+"_m"+str(self.mode)+"_g"+str(self.game_num)+"_t"+str(self.time_label)
                    filename = f"recorded_frames/{folder}/gamescreen/g_{frame_number:05d}_m{millis}.jpg"
                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))
                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(e)
                print("error")


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

