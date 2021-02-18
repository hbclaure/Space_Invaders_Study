import logging
import tornado.escape
import tornado.ioloop
import tornado.options
import tornado.web
import tornado.websocket
import tornado.gen

import os
import uuid
import json
import sqlite3
import random

from agents.cooperative import Cooperative

WEBROOT = os.path.dirname(os.path.realpath(__file__))
DATABASE = os.path.join(WEBROOT, 'db/game_logs.db')

current_agent = Cooperative()

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
            (r"/(.*)", tornado.web.StaticFileHandler, dict(path=settings['static_path'],default_filename="index.html"))
        ]
        super().__init__(handlers, **settings)

class ControlHandler(tornado.websocket.WebSocketHandler):
    def check_origin(self, origin):
        '''Allow from all origins'''
        return True

    def on_message(self, msg):
        # TODO: do something with state
        state = json.loads(msg)

        ## or abstract the agent out into another object
        action = current_agent.update(state)

        ## or a nn agent
        #output = torchmodel.inference(state)
        #action = output

        # or random agent
        #action = {
        #    'left': False if random.randint(0,1) == 0 else True,
        #    'right': False if random.randint(0,1) == 0 else True,
        #    'shoot': False if random.randint(0,1) == 0 else True
        #}

        # send a smarter (non-random) action
        self.write_message(json.dumps(action))


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
    app.listen(8888, '0.0.0.0')
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()

