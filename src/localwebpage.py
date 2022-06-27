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

from tornado.options import define, options
define("port",default = 8282, help="run on the given port", type=int)
define("machine",default='anna',help="run on machine",type=str)

WEBROOT = os.path.dirname(os.path.realpath(__file__))

class Application(tornado.web.Application):
    '''
    ROS node
    '''
    def __init__(self):
        # Initialize the node
        print("initializing local webpage node")
        rospy.init_node('localwebpage')

        handlers = [
            (r"/robot_wake", RobotWakeHandler),
            (r"/robot_introduction", RobotIntroHandler),
            (r"/robot_sleep",RobotSleepHandler)
        ]
        super().__init__(handlers)

class RobotWakeHandler(tornado.websocket.WebSocketHandler):
    # put in __init__
    def __init__(
        self,
        application: tornado.web.Application,
        request: tornado.httputil.HTTPServerRequest,
        **kwargs: Any
    ) -> None:
        super().__init__(application, request, **kwargs)        

        # Publisher
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)
    
    def get(self):
        self.robot_action_pub.publish("wake")

class RobotIntroHandler(tornado.websocket.WebSocketHandler):
    # put in __init__
    def __init__(
        self,
        application: tornado.web.Application,
        request: tornado.httputil.HTTPServerRequest,
        **kwargs: Any
    ) -> None:
        super().__init__(application, request, **kwargs)        

        # Publisher
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)
    
    def get(self):
        self.robot_action_pub.publish("introduction")

class RobotSleepHandler(tornado.websocket.WebSocketHandler):
    # put in __init__
    def __init__(
        self,
        application: tornado.web.Application,
        request: tornado.httputil.HTTPServerRequest,
        **kwargs: Any
    ) -> None:
        super().__init__(application, request, **kwargs)        

        # Publisher
        self.robot_action_pub = rospy.Publisher('space_invaders/game/robot_action',String,queue_size=5)
    
    def get(self):
        self.robot_action_pub.publish("sleep")


def main():
    app = Application()
    tornado.options.parse_command_line()
    port = options.port
    machine = options.machine
    proto = 'http'
    
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
