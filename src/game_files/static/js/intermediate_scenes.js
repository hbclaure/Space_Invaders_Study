// --- Start Screen of the game ---
var start_scene = new Phaser.Scene('start_scene');

// load fonts
start_scene.preload = function() {
    this.load.setBaseURL(static_url + '/');
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
    this.load.bitmapFont('PressStart2P_Blue', 'assets/fonts/PressStart2P_Blue/font.png', 'assets/fonts/PressStart2P_Blue/font.fnt');
}

start_scene.create = function() {
    var game_name = this.add.bitmapText(this.sys.canvas.width / 2, 175, 'PressStart2P_Orange', 'SPACE INVADERS', 50).setOrigin(0.5);
    var instructions = this.add.bitmapText(this.sys.canvas.width / 2, 300, 'PressStart2P_White', 'left/right arrow keys to move, \nspacebar to fire \n \n up/down arrow keys to give\npositive/negative feedback \nto teammate', 23).setOrigin(0.5).setCenterAlign();
    var start = this.add.bitmapText(this.sys.canvas.width / 2, 400, 'PressStart2P_Green', 'Press spacebar to begin', 20).setOrigin(0.5);

    if (mode == PRACTICE) {
        start.setText('Press spacebar to begin tutorial');
    }

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
    //sockets.image.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,display_vid:display_vid}));
    //sockets.game.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,display_vid:display_vid}));
    //save_image_loop(stage=0);

    //UNCOMMENT FOR RECORDING
    waitForSocketConnection(sockets.image, function(){
        console.log("image open");
        socket_start_image();
    });
}

// Make the function wait until the connection is made...
function waitForSocketConnection(socket, callback){
    setTimeout(
        function () {
            if (socket.readyState === 1) {
                console.log("Connection is made");
                if (callback != null){
                    callback();
                }
            } else {
                console.log("wait for connection...");
                waitForSocketConnection(socket, callback);
            }

        }, 5); // wait 5 milisecond for the connection...
}

function socket_start_image() {
    // if(sockets.game.readyState==1){
    //     socket_start();
    // } else if(sockets.game.readyState == 0) {
    //     sockets.game.onopen = () => socket_start();
    // }
    waitForSocketConnection(sockets.game, function(){
        console.log("game open");
        socket_start();
    });
}

function socket_start() {
    sockets.image.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,game_condition:game_condition,display_vid:display_vid}));
    sockets.game.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,game_condition:game_condition,display_vid:display_vid}));
    console.log('starting to record');
    save_image_loop(stage=0);
}

start_scene.update = function() {

    // begin game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        if (mode == PRACTICE) {
            waitForSocketConnection(sockets.control, function(){
                console.log("control open");
                sockets.control.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,game_condition:game_condition,display_vid:display_vid}));
            });
            //sockets.control.send(JSON.stringify(mode))
            // this.scene.start('practice_scene');
            this.scene.start('practice_scene');
            //begin recording frames
            waitForSocketConnection(sockets.image, function(){
                console.log("image open 2");
                waitForSocketConnection(sockets.game, function(){
                    console.log("game open 2");
                    save_image_loop();
                })
            })
        }
        else {
            waitForSocketConnection(sockets.control, function(){
                console.log("control open");
                sockets.control.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,game_condition:game_condition,display_vid:display_vid}));
            });
            //sockets.control.send(JSON.stringify(mode))
            this.scene.start('game_scene');
            // begin recording frames
            waitForSocketConnection(sockets.image, function(){
                console.log("image open 2");
                waitForSocketConnection(sockets.game, function(){
                    console.log("game open 2");
                    save_image_loop();
                })
            })
        }
    }
}


// --- First Intermediate Screen of the game (after practice) ---
var intermediate_scene = new Phaser.Scene('intermediate_scene');

// load fonts
intermediate_scene.preload = function() {
    this.load.setBaseURL(static_url + '/');
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
    this.load.bitmapFont('PressStart2P_Purple', 'assets/fonts/PressStart2P_Purple/font.png', 'assets/fonts/PressStart2P_Purple/font.fnt');
    this.load.bitmapFont('PressStart2P_Gray', 'assets/fonts/PressStart2P_Gray/font.png', 'assets/fonts/PressStart2P_Gray/font.fnt');
    this.load.bitmapFont('PressStart2P_Blue', 'assets/fonts/PressStart2P_Blue/font.png', 'assets/fonts/PressStart2P_Blue/font.fnt');
}

intermediate_scene.create = function() {
    player_score = player_ship.sprite.props.score;
    ai_score = ai_ship.sprite.props.score;

    var gameover_text = this.add.bitmapText(this.sys.canvas.width / 2, 175, 'PressStart2P_Gray', 'ROUND ENDED', 50).setOrigin(0.5);
    var player_text = this.add.bitmapText(this.sys.canvas.width / 2, 300, 'PressStart2P_Purple', 'Player Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    var font_type = (mode == UNCOOPERATIVE) ? 'PressStart2P_Orange' : 'PressStart2P_Gray';
    var ai_text = this.add.bitmapText(this.sys.canvas.width / 2, 400, 'PressStart2P_Orange', 'Co-Player Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var instructions = this.add.bitmapText(this.sys.canvas.width / 2, 500, 'PressStart2P_Green', 'Press spacebar to begin second round', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

intermediate_scene.update = function() {
    // begin second game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        this.scene.start('game_scene');
    }
}

// --- Game Over Screen ---
var gameover_scene = new Phaser.Scene('gameover_scene');

// load fonts
gameover_scene.preload = function () {
    this.load.setBaseURL(static_url + '/');
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
    this.load.bitmapFont('PressStart2P_Purple', 'assets/fonts/PressStart2P_Purple/font.png', 'assets/fonts/PressStart2P_Purple/font.fnt');
    this.load.bitmapFont('PressStart2P_Gray', 'assets/fonts/PressStart2P_Gray/font.png', 'assets/fonts/PressStart2P_Gray/font.fnt');
    this.load.bitmapFont('PressStart2P_Blue', 'assets/fonts/PressStart2P_Blue/font.png', 'assets/fonts/PressStart2P_Blue/font.fnt');
}

// display Game Over and final scores
gameover_scene.create = function() {
    save_image_loop(2);
    player_score = player_ship.sprite.props.score;
    ai_score = ai_ship.sprite.props.score;

    // 4 digit random number
    /*
    var completion_code_num = Math.floor(Math.random() * 899) + 100;
    if(mode ==1){
        var completion_code = completion_code_num.toString()+'e'
    } else if(mode==2){
        var completion_code = completion_code_num.toString()+'l'
    } else if(mode==3){
        var completion_code = completion_code_num.toString()+'u'
    } else{
        var completion_code = completion_code_num.toString()+'o'
    } */

    var gameover_text = this.add.bitmapText(this.sys.canvas.width / 2, 125, 'PressStart2P_Orange', 'GAME ENDED', 50).setOrigin(0.5);
    // var player_text = this.add.bitmapText(400, 250, 'PressStart2P_Purple', 'Player Final Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    // var font_type = (mode == UNCOOPERATIVE) ? 'PressStart2P_Orange' : 'PressStart2P_Gray';
    // var ai_text = this.add.bitmapText(400, 350, font_type, 'AI Final Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var font_type = 'PressStart2P_Green';
    var final_score_text = this.add.bitmapText(this.sys.canvas.width / 2, 300, font_type, 'Final Score: ' + total_score, 30).setOrigin(0.5).setCenterAlign();
    // var cc_text = this.add.bitmapText(400, 450, 'PressStart2P_White', 'Completion Code:', 20).setOrigin(0.5).setCenterAlign();
    // var cc = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Loading...', 20).setOrigin(0.5).setCenterAlign();
    // log this game
    //sockets.log.onmessage = function(event) {
    sockets.control.onmessage = function(event) {
        // if message == "saved", then do this; otherwise do nothing
        if(event.data=="saved"){
            cc.destroy();
            // gameover_scene.add.bitmapText(400, 500, 'PressStart2P_Green', completion_code, 40).setOrigin(0.5).setCenterAlign();
        }  
    }
    sockets.control.send(JSON.stringify(game_log));
    //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
}
