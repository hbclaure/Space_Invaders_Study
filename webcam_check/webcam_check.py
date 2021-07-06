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
from cv2 import CascadeClassifier, imread, imdecode
import numpy as np


from tornado.options import define, options
define("port",default = 8778,help="run on the given port", type=int)
define("machine",default='anna',help="run on machine",type=str)

WEBROOT = os.path.dirname(os.path.realpath(__file__))
DATABASE = os.path.join(WEBROOT, 'db/game_logs.db')
SSL_ROOT = "/home/si_app/ssl"



machines = {
    'anna': ("anna.cs.yale.edu.crt","anna.cs.yale.edu.key"),
    'xpm': ("apache.crt","apache.key")
}

classifier = CascadeClassifier(f"classifiers/haarcascade_frontalface_default.xml")

class Application(tornado.web.Application):
    def __init__(self):
        settings = dict(
            cookie_secret="b0ds8fhasldkgakl1joasdig0asdhgalkj30asgjdgla;ksd;glah",
            template_path=os.path.join(os.path.dirname(__file__), "templates"),
            static_path=os.path.join(os.path.dirname(__file__), "static"),
            xsrf_cookies=True,
        )
        handlers = [
            (r"/image", ImageHandler),
            (r"/(.*)", tornado.web.StaticFileHandler, dict(path=settings['static_path'],default_filename="index.html"))
        ]
        super().__init__(handlers, **settings)





class ImageHandler(tornado.websocket.WebSocketHandler):

    player_id = None
    time_label = None
    count = 0
    box_count = 0

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
                time = datetime.utcnow()
                self.time_label = str(time.year)+"_" + str(time.month)+"_" + str(time.day)+"_" + str(time.hour)+"_" + str(time.minute)
                print(f"Player: {self.player_id}")
            except Exception as e:
                print(e)
                print(state)
                print("invalid")
        else:
            try:
                self.count +=1
                r_msg = json.loads(msg)
                image = base64.b64decode(r_msg['img'].split('base64')[-1])
                millis = r_msg['millis']
                if image:
                    np_img = np.frombuffer(image, dtype=np.uint8)
                    pixels = imdecode(np_img, flags=1)
                    bboxes = classifier.detectMultiScale(pixels, minNeighbors=5)
                    if len(bboxes)==1:
                        self.box_count += 1
                    for box in bboxes:
                        box_msg = {'x1': int(box[0]), 'y1': int(box[1]), 'width': int(box[2]), 'height': int(box[3]),'enough': self.box_count>50,'too_many':len(bboxes)>1}
                        self.write_message(json.dumps(box_msg))
                    folder = "P"+str(self.player_id)+"_t"+str(self.time_label)
                    filename = f"check_data/recorded_frames/{folder}/check/w_{self.count:05d}_m{millis}.jpg"

                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))

                    with open(filename, "+wb") as f:
                        f.write(image)
            except Exception as e:
                print(e)
                print("error")
                #sentry_sdk.capture_exception(e)


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
        app.listen(port, '0.0.0.0')
    print(f"Listening on {proto}://0.0.0.0:%i" % port)
    tornado.ioloop.IOLoop.current().start()

if __name__ == "__main__":
    main()
