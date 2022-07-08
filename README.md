Multi-Agent Space Invaders (Web Version)
----------------------------------------

Space invaders game built in javascript, HTML 5, and [WebSockets](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket) using [Tornado](https://www.tornadoweb.org/en/stable/) and [Phaser 3](https://phaser.io/phaser3). 


## Install

Follow the instructions in [docs/install_osx.md](docs/install_osx.md) to install dependencies in OSX.

## Start the game

```bash
pipenv run python space_invaders/websocket.py
```

## Code Organization

The code is organized as follows:

```text
space_invaders_web/
|   README.md
|-- docs/
    |   install_osx.md          # installation instructions
|-- space_invaders/             # main code
    |   websocket.py            # tornado application
    |-- db/                     # database files
        |   create_databse.sql  # how to create the database
        |   game_logs.db        # sqlite3 database of game logs
    |-- static/					# static files for flask
    	|	assets/				# images, sounds, and fonts for game
    	|	favicon.ico         # favicon for website
    	|   js/					# javascript code
    |-- templates/              # templates for rendering website
        |   index.html          # main website
```

## NGINX notes 

sudo ln -sf /home/si_app/implicit_feedback_si/nginx_config/nginx.conf /etc/nginx/sites-enabled/space_invaders

sudo service nginx restart
sudo service nginx status

sudo ln -sf /home/si_app/implicit_feedback_si/nginx_config/space_invaders.service /etc/systemd/system/space_invaders.service

## Local Webpage
Launch Webpage
```bash
cd error_recovery_si_nao/local_webpage
python3 -m http.server <port number>
rosrun error_recovery_si_nao localwebpage.py
```

Launch Game
```bash
source ../../devel/setup.bash
source venv_game/bin/activate
rosrun error_recovery_si_nao websocket.py
```

Launch Robot controls

Launch Webcam check
```bash
python3 webcam_check/webcam_check.py
```



///
Topics:
/audio/audio /audio/audio_info /body_tracking_data /rgb/camera_info /rgb/image_raw/compressed /space_invaders/game/game_condition /space_invaders/game/game_mode /space_invaders/game/game_state /space_invaders/game/nao_action /space_invaders/game/robot_action /tf /tf_static



roslaunch audio_play play.launch
rosrun image_view image_view image:=/rgb/image_raw _image_transport:=compressed


rosbag record /audio/audio /audio/audio_info /body_tracking_data /rgb/camera_info /rgb/image_raw/compressed /space_invaders/game/game_condition /space_invaders/game/game_mode /space_invaders/game/nao_action /space_invaders/game/robot_action /tf /tf_static -O pilot_ah.bag