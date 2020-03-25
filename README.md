Multi-Agent Space Invaders (Web Version)
----------------------------------------

Space invaders game built in javascript and HTML 5 using [Electron](https://www.electronjs.org/) and [Phaser 3](https://phaser.io/phaser3). 


## Install

Follow the instructions in [docs/install_osx.md](docs/install_osx.md) to install dependencies in OSX.

## Start the game

```bash
$ cd space_invaders_web # this repository
$ cd space_invaders
$ npm start
```

## Code Organization

The code is organized as follows:

```text
space_invaders_web/
|   README.md
|   
|-- space_invaders/             # main code
    |   index.html              # website home
    |   index.js                # main electron JS
    |   package.json            # manifest about the application
    |   package-lock.json       # version of every package nneeded for teh application
    |   assets/                 # static resources for the game (images, sounds)
    |   ... 
    |
    |-- js/                     # javascript code
        | space_invaders.js     # main game logic
```

