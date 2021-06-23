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
    // a log of all of the frames of the game
    frames = [];
    frame_number = 0;
    date = new Date();
    events = [];

  // flag to tell when the game is over
    gameover = false;

    // debug_text flag to run debugging text in developer tools
    debug_text = false;

    cursors = this.input.keyboard.createCursorKeys();
    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);

    this.custom_sounds = {};   // add sound object so that we can easily access the sounds in the scene  
    this.custom_sounds.enemy_explosion = this.sound.add("audio_explosion", {volume: 0.05});
    this.custom_sounds.player_explosion = this.sound.add("audio_explosion", {volume: 0.1});
    this.custom_sounds.fire_ship = this.sound.add("audio_fire_ship", {volume: 0.05});


    player_ship = this.create_ship("ship", 0, this.sys.canvas.width / 2, 540);

    enemies_practice = this.create_enemies(5, 235, 0, "p", 5, 10, "enemylaser", min_x = 205, max_x = 595, [1, 1, 1]);
    

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
         player_ship.sprite.props.score += enemy.score;
         player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score); 	
           events.push({frame: frame_number, killer: 'PLAYER', killed: 'LEFT'});
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
        else if (ship_sprite.props.lives >= 1) {
            events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 4;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                // give the player 50 frames of invincibility
                ship_sprite.props.invincible = true;
                ship_sprite.props.invincibility_timer = 50;
            });
        }
        else {
            events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
            gameover = true;
        }
    });

    // --> enemies hit player_ship
    this.physics.add.collider(player_ship.sprite, enemies_practice.enemies_group, (ship_sprite, enemy) => {
        // play sounds
        this.custom_sounds.player_explosion.play();
        this.custom_sounds.enemy_explosion.play();
        // update the score
        ship_sprite.props.score += enemy.score;
        ship_sprite.props.scoreText.setText("SCORE " + ship_sprite.props.score);
        // kill the player and the enemy. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives >= 1) {
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
            gameover = true;
        }
    });
}


function update_practice_scene() {
    var player_shoots = false;
    var player_bullet = player_ship.bullets_group.getChildren()[0];
    var player_bullet_position = [];

    // make sure player can only have one active bullet at a time
    if (player_bullet.active) {
        player_bullet_position = [player_bullet.body.x, player_bullet.body.y];
    }
    else if (this.input.keyboard.checkDown(space_key, 500)) {
        player_shoots = true;
    }

    player_ship.update(cursors.left.isDown, cursors.right.isDown, player_shoots);

    var enemies_practice_sprites = enemies_practice.enemies_group.getChildren();
    var enemies_positions = [];
    for (var i=0; i < enemies_practice_sprites.length; i++) {
        enemies_positions.push([enemies_practice_sprites[i].x, enemies_practice_sprites[i].y]);
        if (enemies_practice_sprites[i].y > player_ship.sprite.y) {
            gameover = true;
        }
    }

    var enemies_bullets = enemies_practice.bullets_group.getChildren();
    var bullets_positions = []; // for logging purposes
    for (var i = 0; i < enemies_bullets.length; i++) {
        var current_bullet = enemies_bullets[i];
        
        if (current_bullet.active) {
            bullets_positions.push([current_bullet.x, current_bullet.y]);
        }
    }

    enemies_practice.update();


    // log this frame
    var log_frame = {
        frame_number: frame_number,                          //!< Number of the frame
        player_position: player_ship.sprite.x,               //!< Player's position
        player_lives: player_ship.sprite.props.lives,        //!< Player's lives
        player_score: player_ship.sprite.props.score,        //!< Player's score
        player_bullet_position: player_bullet_position,      //!< Positions of all of player's bullets

        ai_position: null,                                   //!< AI Ship's position 
        ai_lives: null,                                      //!< AI Ship's lives
        ai_score: null,                                      //!< AI Ship's score
        ai_bullet_position: [],                              //!< Positions of all of AI player's bullets

        enemies_left_positions: enemies_positions,           //!< Left side enemies' positions (all enemies in practice mode)
        bullets_left_positions: bullets_positions,           //!< Positions of all left side enemies' bullets

        enemies_right_positions: [],                         //!< Right side enemies' positions (none in practice mode)
        bullets_right_positions: [],                         //!< Positions of all right side enemies' bullets
    }

    frames.push(log_frame);
    frame_number += 1;

    // switch to gameover screen
    if (gameover || enemies_practice_sprites.length == 0) {
        game_log = {player_id: player_id, date: date, events: events, frames: frames};
        this.scene.start('gameover_scene_practice');
    }
}

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


    player_ship = this.create_ship("ship", 0, this.sys.canvas.width / 2, 540);
}


function update_tutorial_scene() {
    // --- player shooting logic ---
    var player_tried_to_shoot = false;
    var player_shoots = false;
    var time_since_last_shot = Date.now() - player_ship.sprite.props.last_shot_time;

    if (this.input.keyboard.checkDown(space_key, 0)) {
        player_tried_to_shoot = true;
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

