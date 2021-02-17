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

