// --- Start Screen of the game ---
var start_scene = new Phaser.Scene('start_scene');

// load fonts
start_scene.preload = function() {
    this.load.setBaseURL(static_url + '/');
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
    this.load.image('ship', 'assets/images/ship.png');
    this.load.image('ship2', 'assets/images/ship2.png');


}

start_scene.create = function() {
    var game_name = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'SPACE INVADERS', 50).setOrigin(0.5);
    var instructions;
    var start;
    // if (gamepad1 == true || gamepad2 == true) {

    if (mode == 0){
        instructions = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Observe How Nao Plays', 23).setOrigin(0.5).setCenterAlign();
        start = this.add.bitmapText(400, 400, 'PressStart2P_Green', '', 20).setOrigin(0.5);



    }
    else{
    instructions = this.add.bitmapText(400, 300, 'PressStart2P_White', 'left/right arrow buttons to move, \npress A to fire \n \n ', 23).setOrigin(0.5).setCenterAlign();
    start = this.add.bitmapText(400, 400, 'PressStart2P_Green', 'Press A to begin', 20).setOrigin(0.5);
    }
    // this.load.setBaseURL(static_url + '/');
    // } else {
    //     instructions = this.add.bitmapText(400, 300, 'PressStart2P_White', 'left/right arrow keys to move, \nspacebar to fire \n \n highest score wins', 23).setOrigin(0.5).setCenterAlign();
    //     start = this.add.bitmapText(400, 400, 'PressStart2P_Green', 'Press spacebar to begin', 20).setOrigin(0.5);
    // }

    var orangeShip = this.physics.add.staticSprite(this.sys.canvas.width / 2 - 300, 500, 'ship');
    var blueShip = this.physics.add.staticSprite(this.sys.canvas.width / 2 + 300, 500, 'ship2');

    p1_connected_text = this.add.bitmapText(this.sys.canvas.width / 2 - 300, 550, 'PressStart2P_White', 'Connected', 20).setOrigin(0.5).setVisible(false);
    p2_connected_text = this.add.bitmapText(this.sys.canvas.width / 2 + 300, 550, 'PressStart2P_White', 'Connected', 20).setOrigin(0.5).setVisible(false);
    

    // if (mode == PRACTICE) {
    //     start.setText('Press spacebar to begin tutorial');
    // }
    if (mode == 0 || mode == 1) {
        timer = 15;
    } else {
        timer = 120;
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
    sockets.image.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,display_vid:display_vid, game_condition:game_condition}));
    sockets.game.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,display_vid:display_vid,game_condition:game_condition}));
    console.log('starting to record');
    save_image_loop(stage=0);
}

start_scene.update = function() {

    // begin game when the player presses spacebar or xbox controller A button
    if (this.input.keyboard.checkDown(space_key, 500) || p1xBoxA == true || p2xBoxA == true) {
        if (mode == PRACTICE) {
            waitForSocketConnection(sockets.control, function(){
                console.log("control open");
                sockets.control.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,display_vid:display_vid,game_condition:game_condition,timer:timer}));
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
                sockets.control.send(JSON.stringify({player_id:player_id,mode:mode,game_num:game_num,display_vid:display_vid,game_condition:game_condition,timer:timer}));
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
    // fullscreen
    keyF = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.F);
    keyF.on('down', function () {
        if (this.scale.isFullscreen) {
            this.scale.stopFullscreen();
        } else {
            this.scale.startFullscreen();
        }
    }, this);
    // if (p1xBoxY == true || p2xBoxY == true) {
    //     if (this.scale.isFullscreen) {
    //         this.scale.stopFullscreen();
    //     } else {
    //         this.scale.startFullscreen();
    //     }
    // }

    if (gamepad1 == true) {
        p1_connected_text.setVisible(true);
    } 
    if (gamepad2 == true) {
        p2_connected_text.setVisible(true);
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
}

intermediate_scene.create = function() {
    player_score = player_ship.sprite.props.score;
    ai_score = ai_ship.sprite.props.score;

    var gameover_text = this.add.bitmapText(400, 175, 'PressStart2P_Gray', 'ROUND ENDED', 50).setOrigin(0.5);
    var player_text = this.add.bitmapText(400, 300, 'PressStart2P_Purple', 'Player Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    var font_type = (mode == UNCOOPERATIVE) ? 'PressStart2P_Orange' : 'PressStart2P_White';
    var ai_text = this.add.bitmapText(400, 400, 'PressStart2P_Orange', 'Co-Player Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var instructions = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Press spacebar to begin second round', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

intermediate_scene.update = function() {
    // begin second game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        this.scene.start('game_scene');
    }
}
// intermediate_scene.update2 = function() {
//     // begin second game when the player presses spacebar
//     if (this.input.keyboard.checkDown(space_key, 500)) {
//         this.scene.start('game_scene');
//     }
// }

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
}

// display Game Over and final scores
gameover_scene.create = function() {
    save_image_loop(2);
    player_score = players[0].sprite.props.score + players[1].sprite.props.score;
    ai_score = ai_ship.sprite.props.score;

    // 4 digit random number
    var completion_code_num = Math.floor(Math.random() * 899) + 100;
    if(mode ==1){
        var completion_code = completion_code_num.toString()+'e'
    } else if(mode==2){
        var completion_code = completion_code_num.toString()+'l'
    } else if(mode==3){
        var completion_code = completion_code_num.toString()+'u'
    } else{
        var completion_code = completion_code_num.toString()+'o'
    }

    var gameover_text = this.add.bitmapText(400, 125, 'PressStart2P_Orange', 'ROUND ENDED', 50).setOrigin(0.5);
    // var player_text = this.add.bitmapText(400, 250, 'PressStart2P_Purple', 'Player Final Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    // var font_type = (mode == UNCOOPERATIVE) ? 'PressStart2P_Orange' : 'PressStart2P_Gray';
    // var ai_text = this.add.bitmapText(400, 350, font_type, 'AI Final Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var font_type = 'PressStart2P_Green';
    var final_score_text = this.add.bitmapText(400, 300, font_type, 'Your Score: ' + players[0].sprite.props.score + '\nShutter\'s Score: ' + players[1].sprite.props.score, 30).setOrigin(0.5).setCenterAlign();
    var cc_text = this.add.bitmapText(400, 450, 'PressStart2P_White', 'Press B On the controller to continue', 20).setOrigin(0.5).setCenterAlign();
    //var cc = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Loading...', 20).setOrigin(0.5).setCenterAlign();
    // log this game
    //sockets.log.onmessage = function(event) {
    // sockets.control.onmessage = function(event) {
    //     // if message == "saved", then do this; otherwise do nothing
    //     if(event.data=="saved"){
    //         cc.destroy();
    //         gameover_scene.add.bitmapText(400, 500, 'PressStart2P_Green', completion_code, 40).setOrigin(0.5).setCenterAlign();
    //     }  
    // }
    sockets.control.send(JSON.stringify(game_log));
    //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
}
