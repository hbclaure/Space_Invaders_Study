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

from agents.uncooperative import Uncooperative
from agents.cooperative_early import CooperativeEarly
from agents.cooperative_late import CooperativeLate

WEBROOT = os.path.dirname(os.path.realpath(__file__))
DATABASE = os.path.join(WEBROOT, 'db/game_logs.db')
SSL_ROOT = "/etc/apache2/ssl"

current_agent = CooperativeEarly()

class Application(tornado.web.Application):
    def __init__(self):
        settings = dict(
            cookie_secret="a0ds8fhasldkgakl1joasdig0asdhgalkj30asgjdgla;ksd;glah",
            template_path=os.path.join(os.path.dirname(__file__), "templates"),
            static_path=os.path.join(os.path.dirname(__file__), "static"),
            xsrf_cookies=True,
        )
        handlers = [
            (r"/log", LogHandler),
            (r"/control", ControlHandler),
            (r"/image", ImageHandler),
            (r"/game", GameHandler),
            (r"/(.*)", tornado.web.StaticFileHandler, dict(path=settings['static_path'],default_filename="index.html"))
        ]
        super().__init__(handlers, **settings)

class ControlHandler(tornado.websocket.WebSocketHandler):
    agent_mode = None
    logs = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def log_data(self):
        con = sqlite3.connect(DATABASE)
        cur = con.cursor()

        current_event = 0

        cur.execute('INSERT INTO Games(player_id, date, mode) VALUES(?,?,?)',
                                   (self.player_id, self.date, self.agent_mode))

        game_id = cur.lastrowid

        for frame in self.logs:
            # log the actual frame and collect the id
            cur.execute('''INSERT INTO Frames(game_id, frame_number, timestamp, player_position, player_lives,
                        player_score, ai_position, ai_lives, ai_score) VALUES(?,?,?,?,?,?,?,?,?)''',
                            (game_id, frame['frame_number'], frame['hms'], frame['player_position'], frame['player_lives'],
                            frame['player_score'], frame['ai_position'], frame['ai_lives'], frame['ai_score']))
            frame_id = cur.lastrowid;

            # log the player bullet if it exists
            if (len(frame['player_bullet_position']) > 0):
                cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Player\',?,?,?)',
                                (frame_id, frame['player_bullet_position'][0], frame['player_bullet_position'][1]))

            # log the ai bullet if it exists
            if (len(frame['ai_bullet_position']) > 0):
                cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'AI\',?,?,?)',
                                (frame_id, frame['ai_bullet_position'][0], frame['ai_bullet_position'][1]))

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

            cur.execute('INSERT INTO Actions(frame_id, timestamp, left, right, shoot) VALUES(?,?,?,?,?)',
                            (frame_id, frame['hms'], frame['action']['left'], frame['action']['right'], frame['action']['shoot']))
        con.commit()
        con.close()
        # self.write_message('saved')

    def on_message(self, msg):
        time = datetime.now()
        hms = 'w_'+ str(time.hour)+ "_" + str(time.minute)+ "_" + str(time.second)+ "_" + str(time.microsecond)

        if not self.agent_mode:
            try:
                self.agent_mode = int(json.loads(msg))
                print("Mode: ", self.agent_mode)
                self.logs = []
            except Exception as e:
                print(e)
        else:
            # TODO: do something with state
            state = json.loads(msg)

            ## or abstract the agent out into another object
            if "frame_number" in state.keys():
                action = current_agent.update(state)

                state['action'] = action
                state['hms'] = hms
                self.logs.append(state)

                # send a smarter (non-random) action
                self.write_message(json.dumps(action))
            else:
                log = json.loads(msg)
                self.events = log['events']
                self.player_id = log['player_id']
                self.date = log['date']
                print("events received")

                try:
                    self.log_data()
                    print(f"Saved to database: {self.player_id}")
                except Exception as e:
                    print("Logging error")
                    print(e)



class ImageHandler(tornado.websocket.WebSocketHandler):

    frame_count = 0
    player_id = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        pass

    def on_close(self):
        pass

    def on_message(self, msg):
        time = datetime.now()
        playerid = self.player_id
        id_time = str(playerid)+"_"+str(time.year)+"_" + str(time.month)+"_" + str(time.day)
        hms = 'w_'+ str(time.hour)+ "_" + str(time.minute)+ "_" + str(time.second)+ "_" + str(time.microsecond)
        if not self.player_id:
            try:
                self.player_id = json.loads(msg)
                print(f"Player: {self.player_id}")
            except Exception as e:
                print(e)
                print("invalid")
        else:
            try:
                image = msg
                self.frame_count += 1
                if image:
                    #print("frame {} recorded".format(str(self.frame_count)))
                    filename = f"recorded_frames/{id_time}/webcam/{hms}.jpg"

                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))

                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(e)
                print("error")

class GameHandler(tornado.websocket.WebSocketHandler):
    frame_count = 0
    player_id = None

    def check_origin(self, origin):
        '''Allow from all origins'''
        return True
    
    def open(self):
        pass

    def on_close(self):
        pass

    def on_message(self, msg):
        time = datetime.now()
        playerid = self.player_id
        id_time = str(playerid)+"_"+str(time.year)+"_" + str(time.month)+"_" + str(time.day)
        hms = 'g_'+ str(time.hour)+ "_" + str(time.minute)+ "_" + str(time.second)+ "_" + str(time.microsecond)
        if not self.player_id:
            try:
                self.player_id = json.loads(msg)
                print(self.player_id)
                print(f"Recording frames: {self.player_id}")
            except Exception as e:
                print(e)
                print("invalid")
        else:
            try:
                image = msg
                self.frame_count += 1
                if image:
                    #print("frame {} recorded".format(str(self.frame_count)))
                    filename = f"recorded_frames/{id_time}/gamescreen/{hms}.jpg"

                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))

                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(e)
                print("error")

class LogHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        '''Allow from all origins'''
        return True

    def open(self):
        pass
        #self.write_message('hi')
        #logging.info("connect: there are now %d connections", len(self.waiters))

    def on_close(self):
        pass
        #logging.info("disconnect: there are now %d connections", len(self.waiters))

    def on_message(self, msg):
        logs = json.loads(msg)

        con = sqlite3.connect(DATABASE)
        cur = con.cursor()

        # Go through each game log
        for log in logs:
            # Log the actual game and collect the id

            cur.execute('INSERT INTO Games(player_id, date, round, mode) VALUES(?, ?, ?, ?)',
                                   (log['player_id'], log['date'], log['round'], log['mode']))

            game_id = cur.lastrowid

            # Keep track of the event that we're checking
            current_event = 0;

            # Go through every frame of the game
            for frame in log['frames']:
                # log the actual frame and collect the id
                cur.execute('''INSERT INTO Frames(game_id, frame_number, player_position, player_lives,
                            player_score, ai_position, ai_lives, ai_score) VALUES(?,?,?,?,?,?,?,?)''',
                             (game_id, frame['frame_number'], frame['player_position'], frame['player_lives'],
                              frame['player_score'], frame['ai_position'], frame['ai_lives'], frame['ai_score']))
                frame_id = cur.lastrowid;

                # log the player bullet if it exists
                if (len(frame['player_bullet_position']) > 0):
                    cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Player\',?,?,?)',
                                   (frame_id, frame['player_bullet_position'][0], frame['player_bullet_position'][1]))

                # log the ai bullet if it exists
                if (len(frame['ai_bullet_position']) > 0):
                    cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'AI\',?,?,?)',
                                   (frame_id, frame['ai_bullet_position'][0], frame['ai_bullet_position'][1]))

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
                while (current_event < len(log['events']) and log['events'][current_event]['frame'] == frame['frame_number']):
                    cur.execute('INSERT INTO Events(frame_id, killer, killed) VALUES(?,?,?)',
                                (frame_id, log['events'][current_event]['killer'], log['events'][current_event]['killed']))
                    current_event += 1

        con.commit()
        con.close()

        self.write_message('saved')


def main():
    app = Application()
    port = 8888
    proto = 'http'
    if os.path.exists(SSL_ROOT):
        ssl_ctx = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        ssl_ctx.load_cert_chain(os.path.join(SSL_ROOT, "anna.cs.yale.edu.crt"),
                                os.path.join(SSL_ROOT, "anna.cs.yale.edu.key"))
        server = tornado.httpserver.HTTPServer(app, ssl_options=ssl_ctx)
        proto = 'https'
        server.bind(port)
    else:
        app.listen(port, '0.0.0.0')
    tornado.ioloop.IOLoop.current().start()
    print(f"Listening on {proto}://0.0.0.0:%i" % port)

if __name__ == "__main__":
    main()

