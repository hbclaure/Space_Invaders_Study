// --- Practice session so players can learn controls
var tutorial_scene = {
    preload: preload,
    create: create_tutorial_scene,
    update: update_tutorial_scene,
    extend: {
        fire_bullet: fire_bullet,
        create_bullets_pool: create_bullets_pool,
        create_ship: create_ship,
        create_enemies: create_enemies,
    }
};

function create_tutorial_scene() {
    cursors = this.input.keyboard.createCursorKeys();
    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);

    this.custom_sounds = {};   // add sound object so that we can easily access the sounds in the scene  
    this.custom_sounds.enemy_explosion = this.sound.add("audio_explosion", {volume: 0.05});
    this.custom_sounds.player_explosion = this.sound.add("audio_explosion", {volume: 0.1});
    this.custom_sounds.fire_ship = this.sound.add("audio_fire_ship", {volume: 0.05});


    ai_ship = this.create_ship("avery", 0, this.sys.canvas.width / 4 + 400, 540);
    player_ship = this.create_ship("ship", 0, this.sys.canvas.width / 4, 540);
    // enemy = this.create_enemies(1, this.sys.canvas.width/2, 0, "a", 0, 0, "enemylaser", 0, 800, [1,0,0]);

    instruction_num = 1
    instructions = {
        1: "Press left and right to move \n and space bar to shoot",
        2: "Try pressing up to say\nthe orange teammate is doing\na good job",
        3: "Try presisng down to say\nthe orange teammate is doing\na bad job",
        4: "Feel free to proceed\nwhen you are ready"
    }

    instruction_text = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'Try pressing up ', 20).setOrigin(0.5);
    instruction_text.align = 1;

    left_pressed = false;
    right_pressed = false;
    space_pressed = false;
}


function update_tutorial_scene() {
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
        // ai_ship.sprite.props.message.visible = false;

        //player_ship.sprite.props.emote.setFillStyle(0xFFFFFF);
    }

    if (this.input.keyboard.checkDown(cursors.up, 0)) {
        if(frame_number >= last_msg_frame + frames_per_message) {
            console.log('up check pressed');

            player_ship.sprite.props.message.setText("Good job");
            player_ship.sprite.props.message.visible = true;

            // ai_ship.sprite.props.message.text = 'yay';
            // ai_ship.sprite.props.message.visible = true;

            //player_ship.sprite.props.emote.setFillStyle(0x00FF00)
            last_msg_frame = frame_number;

            signal_up = true;
            tried_signal_up = true;
        } else {
            tried_signal_up = true;
        }
    } else if (this.input.keyboard.checkDown(cursors.down, 0)) {
        if (frame_number >= last_msg_frame + frames_per_message) {
            console.log('down check pressed');

            player_ship.sprite.props.message.setText("Bad job");
            player_ship.sprite.props.message.visible = true;

            //player_ship.sprite.props.emote.setFillStyle(0xFF0000)
            // ai_ship.sprite.props.message.text = ai_messages[mode];
            // ai_ship.sprite.props.message.visible = true;
            // ai_ship.sprite.props.message.align = 1;
            last_msg_frame = frame_number;

            signal_down = true;
            tried_signal_down = true;
        } else {
            tried_signal_down = true;
        }
    }

    // update instructions

    if(instruction_num == 1 && space_pressed && left_pressed && right_pressed) {
        instruction_num += 1;
    }

    if(instruction_num == 2 && signal_up) {
        instruction_num += 1;
    }

    if(instruction_num == 3 && signal_down) {
        instruction_num += 1;
    }

    instruction_text.setText(instructions[instruction_num]);

    // player_action = {left: player_left, right: player_right, shoot: player_shoots, tried_to_shoot: player_tried_to_shoot};

    // ai_received_action = {left: ai_commands.left, right: ai_commands.right, shoot: ai_commands.shoot};
    
    // if (ai_ready == false) {
    //     //console.log("waiting: ", frame_number);
    //     //this.scene.pause();
    //     //game_paused = true;
    //     //console.log("PAUSE");
    //     ai_actual_action = {left: false, right: false, shoot: false};
    //     if (frame_number == 0){
    //         frame_sent = true;
    //     } else{
    //         frame_sent = false;
    //     }
        
    // } else if (ai_ready) {
    //     //console.log("** Updating player")
    //     // update player
    player_ship.update(player_left, player_right, player_shoots);
        
    //     // update ai agent, enforce rules of the game
    //     var shoot = ai_commands.shoot && ai_shoots;
    //     var left = ai_commands.left && !ai_commands.right //&& !ai_shoots;
    //     var right = ai_commands.right && !ai_commands.left //&& !ai_shoots;
    //     //console.log("**Updating ai")
    //     ai_ship.update(left, right, shoot);
    //     ai_actual_action = {left: left, right: right, shoot: shoot};

    //     // update the enemies
    //     enemies_left.update();
    //     enemies_right.update();
        
    //     frame_sent = true;
    // }

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

    // ai bullets
    // var ai_bullets = ai_ship.bullets_group.getChildren();
    // var ai_bullets_positions = []; // for logging purposes
    // for (var i = 0; i < ai_bullets.length; i++) {
    //     var current_bullet = ai_bullets[i];
        
    //     if (current_bullet.active) {
    //         ai_bullets_positions.push([current_bullet.x, current_bullet.y]);

    //     }
    // }

    // enemies
    // var enemies_right_sprites = enemies_right.enemies_group.getChildren();
    // var enemies_left_sprites = enemies_left.enemies_group.getChildren();

    // // right side bullets
    // var enemies_right_bullets = enemies_right.bullets_group.getChildren();
    // var bullets_right_positions = []; // for logging purposes
    // for (var i = 0; i < enemies_right_bullets.length; i++) {
    //     var current_bullet = enemies_right_bullets[i];
        
    //     if (current_bullet.active) {
    //         bullets_right_positions.push([current_bullet.x, current_bullet.y]);

    //     }
    // }
    
    // left side bullets
    // var enemies_left_bullets = enemies_left.bullets_group.getChildren();
    // var bullets_left_positions = []; // for logging purposes
    // for (var i = 0; i < enemies_left_bullets.length; i++) {
    //     var current_bullet = enemies_left_bullets[i];
        
    //     if (current_bullet.active) {
    //         bullets_left_positions.push([current_bullet.x, current_bullet.y]);

    //     }
    // }

    // ai-side enemies
    // var enemies_right_positions = [];
    // for (var i=0; i < enemies_right_sprites.length; i++) {
    //     var current_enemy = enemies_right_sprites[i];

    //     enemies_right_positions.push([current_enemy.x, current_enemy.y]);
    //     // end game if enemies reach bottom
    //     if (current_enemy.y > 540) {
    //         ai_over = true;
    //         player_over = true;
    //     }

    // }

    // player-side enemies
    // var enemies_left_positions = [];
    // for (var i=0; i < enemies_left_sprites.length; i++) {
    //     var current_enemy = enemies_left_sprites[i];

    //     enemies_left_positions.push([current_enemy.x, current_enemy.y]);
    //     // end game if enemies reach bottom
    //     if (current_enemy.y > 540) {
    //         player_over = true;
    //         ai_over = true;
    //     }
    // }

    // --- log state
    // var log_frame = {
    //     frame_number: frame_number,                          //!< Number of the frame
    //     timestamp: Date.now(),

    //     player_position: player_ship.sprite.x,               //!< Player's position
    //     player_lives: player_ship.sprite.props.lives,        //!< Player's lives
    //     player_score: player_ship.sprite.props.score,        //!< Player's score
    //     player_bullets_positions: player_bullets_positions,   //!< Positions of all of player's bullets

    //     ai_position: ai_ship.sprite.x,                       //!< AI Ship's position 
    //     ai_lives: ai_ship.sprite.props.lives,                //!< AI Ship's lives
    //     ai_score: ai_ship.sprite.props.score,                //!< AI Ship's score
    //     ai_bullets_positions: ai_bullets_positions,              //!< Positions of all of AI player's bullets

    //     enemies_left_positions: enemies_left_positions,      //!< Left side enemies' positions
    //     bullets_left_positions: bullets_left_positions,      //!< Positions of all left side enemies' bullets

    //     enemies_right_positions: enemies_right_positions,    //!< Right side enemies' positions
    //     bullets_right_positions: bullets_right_positions,    //!< Positions of all right side enemies' bullets

    //     can_shoot: ai_shoots,

    //     player_last_shot_time: player_ship.sprite.props.last_shot_time, // frame when player last shot
    //     ai_last_shot_time: ai_ship.sprite.props.last_shot_time,         // frame when ai last shot
    //     player_last_shot_frame: player_ship.sprite.props.last_shot_frame, // frame when player last shot
    //     ai_last_shot_frame: ai_ship.sprite.props.last_shot_frame,         // frame when ai last shot
    //     player_avg_frequency: average_frequency,

    //     frame_sent: frame_sent,
    //     player_action: player_action,
    //     ai_actual_action: ai_actual_action,
    //     ai_received_action: ai_received_action,

    //     // for error signaling
    //     signal_down: signal_down,
    //     signal_up: signal_up,
    //     // if player pressed but it is during the delay
    //     tried_signal_down: tried_signal_down,
    //     tried_signal_up: tried_signal_up
    // }
    
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

    // --- check game over conditions ---
    // if (enemies_left_sprites.length == 0) {
    //     player_over = true;
    // }
    // if (enemies_right_sprites.length == 0 && (mode == 3|| enemies_left_sprites.length == 0)) {
    //     ai_over = true;
    // }
}

