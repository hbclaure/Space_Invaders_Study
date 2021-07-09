// --- Practice session so players can learn controls
var practice_scene = {
    preload: preload,
    create: create_practice_scene,
    update: update_practice_scene,
    extend: {
        fire_bullet: fire_bullet,
        create_bullets_pool: create_bullets_pool,
        create_ship: create_ship,
        create_enemies: create_enemies,
    }
};

function create_practice_scene() {
    frames = [];
    events = [];
    frame_number = 0;
    last_frame = -1;
    date = new Date();

    cursors = this.input.keyboard.createCursorKeys();
    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
    q_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.Q);
    p_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.P);

    this.custom_sounds = {};   // add sound object so that we can easily access the sounds in the scene  
    this.custom_sounds.enemy_explosion = this.sound.add("audio_explosion", {volume: 0.05});
    this.custom_sounds.player_explosion = this.sound.add("audio_explosion", {volume: 0.1});
    this.custom_sounds.fire_ship = this.sound.add("audio_fire_ship", {volume: 0.05});


    ai_ship = this.create_ship("avery", 0, this.sys.canvas.width / 4 + 400, 540);
    player_ship = this.create_ship("ship", 0, this.sys.canvas.width / 4, 540);

    instruction_num = 1
    instructions = {
        1: "Press left and right to move \n and space bar to shoot",
        2: "Try pressing the up key to say\nthe orange teammate is doing\na good job",
        3: "Try presisng the down key to say\nthe orange teammate is doing\na bad job",
        4: "When you are ready,\npress P to practice with a few enemies.\n\nThe tutorial will end afterwards.",
        5: "",
        6: "When you are done\npracticing the controls,\nclick Q",
    }

    instruction_text = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'Try pressing up ', 20).setOrigin(0.5);
    instruction_text.align = 1;

    left_pressed = false;
    right_pressed = false;
    space_pressed = false;

    up_ready = false;
    down_ready = false;

    gameover = false;
}


function update_practice_scene() {
    // --- player shooting logic ---
    var player_tried_to_shoot = false;
    var player_shoots = false;
    var time_since_last_shot = Date.now() - player_ship.sprite.props.last_shot_time;

    if (this.input.keyboard.checkDown(space_key, 0)) {
        player_tried_to_shoot = true;
        space_pressed = true;
        //console.log("max:",max_player_frequency);
        //console.log("time since:", time_since_last_shot);
        if (time_since_last_shot >= max_player_frequency) {
            player_shoots = true;
            if (previous_shots_time.length == 5) {
                previous_shots_time.shift();
            }
            previous_shots_time.push(time_since_last_shot);
        }
    };

    // rolling average of player ship shot frequency
    total = 0;
    for (i=0; i < previous_shots_time.length; i += 1) {
        total += previous_shots_time[i];
    }
    average_frequency = total / previous_shots_time.length;

    // //// --- AI Logic

    // //// Determine whether ai_ship is able to shoot: true if no AI bullet is active and the shot_cooldown timer has expired
    // var ai_shoots = false;

    // if (Date.now() - ai_ship.sprite.props.last_shot_time >= max_ai_frequency) {
    //     ai_shoots = true;
    // }

    //// --- keep track of actions

    player_left = cursors.left.isDown;
    player_right = cursors.right.isDown;

    if (player_left) {
        left_pressed = true;
    }
    if (player_right) {
        right_pressed = true;
    }

    player_up = cursors.up.isDown;
    player_down = cursors.down.isDown;

    // if(player_up){
    //     console.log('up pressed')
    // }

    // if (player_down){
    //     console.log('down pressed')
    // }

    // reset emote/message after certain amount of time/frames
    if (frame_number >= last_msg_frame + frames_per_message) {
        player_ship.sprite.props.message.visible = false;
        ai_ship.sprite.props.message.visible = false;

        //player_ship.sprite.props.emote.setFillStyle(0xFFFFFF);
    }

    if (this.input.keyboard.checkDown(cursors.up, 0) && !gameover) {
        if(frame_number >= last_msg_frame + frames_per_message) {
            console.log('up check pressed');

            player_ship.sprite.props.message.setText("Good job");
            player_ship.sprite.props.message.visible = true;

            ai_ship.sprite.props.message.text = 'Yay';
            ai_ship.sprite.props.message.visible = true;

            //player_ship.sprite.props.emote.setFillStyle(0x00FF00)
            last_msg_frame = frame_number;

            signal_up = true;
            tried_signal_up = true;
        } else {
            tried_signal_up = true;
        }
    } else if (this.input.keyboard.checkDown(cursors.down, 0) && !gameover) {
        if (frame_number >= last_msg_frame + frames_per_message) {
            console.log('down check pressed');

            player_ship.sprite.props.message.setText("Bad job");
            player_ship.sprite.props.message.visible = true;

            //player_ship.sprite.props.emote.setFillStyle(0xFF0000)
            ai_ship.sprite.props.message.text = "Sorry!"
            ai_ship.sprite.props.message.visible = true;
            ai_ship.sprite.props.message.align = 1;
            last_msg_frame = frame_number;

            signal_down = true;
            tried_signal_down = true;
        } else {
            tried_signal_down = true;
        }
    }

    if (instruction_num == 5) {
        var enemies_practice_sprites = enemies_practice.enemies_group.getChildren();
        for (var i=0; i < enemies_practice_sprites.length; i++) {
            var current_enemy = enemies_practice_sprites[i];
            console.log(current_enemy);
            // end game if enemies reach bottom
            if (current_enemy.y > 540) {
                gameover = true;
            }
        }
        if (enemies_practice.enemies_group.getChildren().length == 0) {
            gameover = true;
        }
    }

    // update instructions

    if (instruction_num == 1 && space_pressed && left_pressed && right_pressed) {
        instruction_num += 1;
        sockets.control.send(JSON.stringify(game_log));
    } else if (instruction_num == 2 && signal_up) {
        up_ready = true;
    } else if (instruction_num == 2 && up_ready && frame_number >= last_msg_frame + frames_per_message) {
        instruction_num += 1;
        sockets.control.send(JSON.stringify(game_log));
    } else if (instruction_num == 3 && signal_down) {
        down_ready = true;
    } else if (instruction_num == 3 && down_ready && frame_number >= last_msg_frame + frames_per_message) {
        instruction_num += 1;
        sockets.control.send(JSON.stringify(game_log));
    } else if (instruction_num == 4 && this.input.keyboard.checkDown(p_key, 500)) {
        sockets.control.send(JSON.stringify(game_log));
        enemies_practice = this.create_enemies(3, this.sys.canvas.width / 2, 0, "p", 5, 10, "enemylaser", min_x = 205, max_x = 595, [1, 1, 1]);
            // --> COLLIDERS <--
        // --> enemies hit by player_ship bullets 
        this.physics.add.collider(enemies_practice.enemies_group, player_ship.bullets_group, (enemy, bullet) => {
            // destroy the enemy
            this.custom_sounds.enemy_explosion.play();
            enemy.play(enemy.explote_anim, true);
            enemy.on('animationcomplete', () => {
                enemy.destroy();
            });
            // hide the bullet 
            bullet.body.x = this.sys.canvas.width;
            bullet.body.y = this.sys.canvas.height;
            bullet.setActive(false);
            // update the score     
            if (enemy.hit == false) {   
            // player_ship.sprite.props.score += enemy.score;
            // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score); 	
            events.push({frame: frame_number, killer: 'PLAYER', killed: 'LEFT', type: 'SHOT'});
            }
            enemy.hit = true;
        });

        // --> enemies bullets hit ships bullets
        this.physics.add.collider(enemies_practice.bullets_group, player_ship.bullets_group, (enemy_bullet, ship_bullet) => {
            // hide both bullets 
            enemy_bullet.body.x = this.sys.canvas.width;
            enemy_bullet.body.y = this.sys.canvas.height;
            ship_bullet.body.x = this.sys.canvas.width;
            ship_bullet.body.y = this.sys.canvas.height;
            enemy_bullet.setActive(false);
            ship_bullet.setActive(false);
        });

        // --> enemies bullets hit player_ship
        this.physics.add.collider(player_ship.sprite, enemies_practice.bullets_group, (ship_sprite, bullet) => {
            // hide the bullet 
            bullet.body.x = this.sys.canvas.width;
            bullet.body.y = this.sys.canvas.height;
            bullet.setActive(false);

            if (ship_sprite.props.invincible) { }
            // kill the player. The change in behavior takes place within the update function of the ship
            else if (ship_sprite.props.lives > 1) {
                ship_sprite.props.invincible = true;
                ship_sprite.props.invincibility_timer = frame_number;
                events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'SHOT'});
                this.custom_sounds.player_explosion.play();
                ship_sprite.props.exploding = true;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
                ship_sprite.play(ship_sprite.explote_anim, true);
                ship_sprite.on('animationcomplete', () => {
                    ship_sprite.x = this.sys.canvas.width / 4;
                    ship_sprite.setTexture(ship_sprite.props.image_id);
                    ship_sprite.props.exploding = false;
                });
            }
            else {
                events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'SHOT'});
                this.custom_sounds.player_explosion.play();
                ship_sprite.props.dead = true;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
                gameover = true;
            }
        });

        // --> enemies hit player_ship
        this.physics.add.collider(player_ship.sprite, enemies_practice.enemies_group, (ship_sprite, enemy) => {
            // play sounds
            this.custom_sounds.player_explosion.play();
            this.custom_sounds.enemy_explosion.play();
            // update the score
            // ship_sprite.props.score += enemy.score;
            // ship_sprite.props.scoreText.setText("SCORE " + ship_sprite.props.score);
            // kill the player and the enemy. The change in behavior takes place within the update function of the ship
            if (ship_sprite.props.lives > 1) {
                ship_sprite.props.exploding = true;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
                ship_sprite.play(ship_sprite.explote_anim, true);
                enemy.play(enemy.explote_anim, true);
                ship_sprite.on('animationcomplete', () => {
                    ship_sprite.x = this.sys.canvas.width / 4;
                    ship_sprite.setTexture(ship_sprite.props.image_id);
                    ship_sprite.props.exploding = false;
                    enemy.destroy();
                });
            }
            else {
                ship_sprite.props.dead = true;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
                gameover = true;
            }
            events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'HIT'});
            events.push({frame: frame_number, killer: 'PLAYER', killed: 'LEFT', type: 'HIT'});
        });
        instruction_num += 1;
    } else if (instruction_num == 5 && gameover) {
        this.scene.start('practice_over_scene');
    } 

    instruction_text.setText(instructions[instruction_num]);

    player_action = {left: player_left, right: player_right, shoot: player_shoots, tried_to_shoot: player_tried_to_shoot};

    // ai_received_action = {left: ai_commands.left, right: ai_commands.right, shoot: ai_commands.shoot};
    
    if (ai_ready == false) {
        if (frame_number == 0){
            frame_sent = true;
        } else{
            frame_sent = false;
        }
    } else if (ai_ready) {
        //console.log("** Updating player")
        // update player
        player_ship.update(player_left, player_right, player_shoots);
        if (instruction_num == 5) {
            enemies_practice.update();
        }
        frame_sent = true;
    }

    // ------ get state information
    // player bullets
    var player_bullets = player_ship.bullets_group.getChildren();
    var player_bullets_positions = []; // for logging purposes
    for (var i = 0; i < player_bullets.length; i++) {
        var current_bullet = player_bullets[i];
        
        if (current_bullet.active) {
            player_bullets_positions.push([current_bullet.x, current_bullet.y]);

        }
    }

    // log state
    var log_frame = {
        frame_number: frame_number,                          //!< Number of the frame
        timestamp: Date.now(),
        frame_sent: frame_sent,

        player_position: player_ship.sprite.x,               //!< Player's position

        player_action: player_action,

        // for error signaling
        signal_down: signal_down,
        signal_up: signal_up,
        // if player pressed but it is during the delay
        tried_signal_down: tried_signal_down,
        tried_signal_up: tried_signal_up
    }
    frames.push(log_frame);

    if (frame_sent){
        sockets.control.send(JSON.stringify(log_frame));
        ai_ready = false;
    }
    
    // --- send state if server was ready
    // if (frame_sent){
    //     sockets.control.send(JSON.stringify(log_frame));
    //     ai_ready = false;
    // }

    // --- log frame of game
    // frames.push(log_frame);
    frame_number += 1;

    signal_down = false;
    signal_up = false;
    tried_signal_down = false;
    tried_signal_up = false;

    game_log = {player_id: player_id, date: date, practice_stage: instruction_num, frames: frames};
    
    // if (instruction_num == 6 && this.input.keyboard.checkDown(q_key, 500)) {
    //     this.scene.start('practice_over_scene');
    // }    
}

// --- Game Over Screen ---
var practice_over_scene = new Phaser.Scene('practice_over_scene');

// load fonts
practice_over_scene.preload = function () {
    this.load.setBaseURL(static_url + '/');
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
    this.load.bitmapFont('PressStart2P_Purple', 'assets/fonts/PressStart2P_Purple/font.png', 'assets/fonts/PressStart2P_Purple/font.fnt');
    this.load.bitmapFont('PressStart2P_Gray', 'assets/fonts/PressStart2P_Gray/font.png', 'assets/fonts/PressStart2P_Gray/font.fnt');
}

// display Game Over and final scores
practice_over_scene.create = function() {
    save_image_loop(2);

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

    var gameover_text = this.add.bitmapText(400, 125, 'PressStart2P_Orange', 'Tutorial Ended', 50).setOrigin(0.5);
    // var player_text = this.add.bitmapText(400, 250, 'PressStart2P_Purple', 'Player Final Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    // var font_type = (mode == UNCOOPERATIVE) ? 'PressStart2P_Orange' : 'PressStart2P_Gray';
    // var ai_text = this.add.bitmapText(400, 350, font_type, 'AI Final Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var font_type = 'PressStart2P_Green';
    // var final_score_text = this.add.bitmapText(400, 300, font_type, 'Final Score: ' + total_score, 30).setOrigin(0.5).setCenterAlign();
    var cc_text = this.add.bitmapText(400, 450, 'PressStart2P_White', 'Completion Code:', 20).setOrigin(0.5).setCenterAlign();
    var cc = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Loading...', 20).setOrigin(0.5).setCenterAlign();
    // log this game
    //sockets.log.onmessage = function(event) {
    sockets.control.onmessage = function(event) {
        // if message == "saved", then do this; otherwise do nothing
        console.log('message received');
        if(event.data=="saved"){
            cc.destroy();
            practice_over_scene.add.bitmapText(400, 500, 'PressStart2P_Green', completion_code, 40).setOrigin(0.5).setCenterAlign();
        }  
    }
    sockets.control.send(JSON.stringify(game_log));
    console.log(completion_code);
    //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
}
