proto = window.location.protocol === 'https:' ? 'wss' : 'ws';

var sockets = {
  // log: new WebSocket(proto + "://"+window.location.host+"/log"),
  control: new WebSocket(proto + "://"+window.location.host+"/control"),
  image: new WebSocket(proto + "://"+window.location.host+"/image"),
  game: new WebSocket(proto + "://"+window.location.host+"/game"),
}

// handle commands from the server
var ai_commands = {
  left: false,
  right: false,
  shoot: false,
};
var ai_ready = false;
sockets.control.onmessage = function(event) {
  var msg = JSON.parse(event.data);
  ai_commands.left = msg.left;
  ai_commands.right = msg.right;
  ai_commands.shoot = msg.shoot;
  ai_ready = true;
  //console.log("RECEIVE", frame_number)
};


var config = {
    type: Phaser.CANVAS, //Phaser.AUTO,
    width: 800,
    height: 600,
    physics: {
        default: 'arcade',
        fps: 30,
        arcade: {
            debug: false, // set to true to enable physics visualization
            gravity: { y: 100 }
        }
    },
};

var game_scene = {
    preload: preload,
    create: create,
    update: update,
    extend: {
        fire_bullet: fire_bullet,
        create_bullets_pool: create_bullets_pool,
        create_ship: create_ship,
        create_enemies: create_enemies,
    }
};


var game = new Phaser.Game(config);     //!< game object

// Add all of the scenes (see start_scene.js and end_scenes.js)
game.scene.add('start_scene', start_scene);
game.scene.add('intermediate_scene', intermediate_scene);
game.scene.add('gameover_scene', gameover_scene);
game.scene.add('gameover_scene_practice', gameover_scene_practice);
game.scene.add('game_scene', game_scene);
game.scene.add('practice_scene', practice_scene);

game.scene.start('start_scene');
