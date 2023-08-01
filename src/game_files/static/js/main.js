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
  support: 0, 

  left_player: false,
  right_player: false,
  shoot_player: false, 
  support_player:0


};
var ai_ready = false;
var send_msg = false;
var frame_sent = false; 
var ai_player_ready = false;
//var game_paused = false;
sockets.control.onmessage = function(event) {
  var msg = JSON.parse(event.data);
  //console.log('received messages', msg)
  ai_commands.left = msg.left;
  ai_commands.right = msg.right;
  ai_commands.shoot = msg.shoot;
  ai_commands.support = msg.support;
  ai_ready = true;

  ai_commands.player_left = msg.player_left; 
  ai_commands.player_right = msg.player_right;
  ai_commands.player_shoot = msg.player_shoot;
  ai_commands.player_support = msg.player_support;
  ai_player_ready = true;



  //if(game_paused){
  //  intermediate_scene.scene.resume('game_scene');
  //  game_paused = false;
  //}
  //console.log("RECEIVE", frame_number)
};


var config = {
    type: Phaser.CANVAS, //Phaser.AUTO,
    // width: 800,
    // width: window.innerWidth,
    // height: 600,
    // height: window.innerHeight,
    scale: {
      mode: Phaser.Scale.FIT,
      parent: 'phaser-example',
      autoCenter: Phaser.Scale.CENTER_BOTH,
      width: 800,
      height: 600
    },
    physics: {
        default: 'arcade',
        arcade: {
            debug: false, // set to true to enable physics visualization
            gravity: { y: 0.001 },
            //fps: 1
        }
    },
};

var game_scene = {
    preload: preload,
    create: create,
    update: update,
    // update: () => update(0),
    // update: update1, 
    // update: update2,
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
game.scene.add('game_scene', game_scene);
game.scene.add('practice_scene', practice_scene);
game.scene.add('practice_over_scene', practice_over_scene)

game.scene.start('start_scene');

