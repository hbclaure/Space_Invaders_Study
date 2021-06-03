/**
 * Update the state of the game
 */
function update ()
{
    // --------- update the ship

    // TODO: sync player ship and ai ship
    var player_shoots = false;
    var player_bullet = player_ship.bullets_group.getChildren()[0];
    var player_bullet_position = [];

    // make sure player can only have one active bullet at a time
    if (player_bullet.active) {
        player_bullet_position = [player_bullet.body.x, player_bullet.body.y];
    }


    var frames_since_last_shot = frame_number - player_ship.sprite.props.last_shot_frame;
    var time_since_last_shot = Date.now() - player_ship.sprite.props.last_shot_time;
    //console.log('frames_since_last_shot', frames_since_last_shot);
    //console.log('time_since_last_shot', time_since_last_shot);
    if (this.input.keyboard.checkDown(space_key, 0)) {
      if (time_since_last_shot >= max_player_frequency) {
        if (player_bullet.active) {
            player_bullet_position = [player_bullet.body.x, player_bullet.body.y];
        }
        player_shoots = true;
        if (previous_shots_time.length == 5) {
            previous_shots_time.shift();
        }
        previous_shots_time.push(time_since_last_shot);
      } else {
        //console.log(`can't shoot, last shot was only ${time_since_last_shot}ms ago`)
      }
    } else if (player_bullet.active) {
        player_bullet_position = [player_bullet.body.x, player_bullet.body.y];
    } else if (this.input.keyboard.checkDown(space_key, 500)) {
        player_shoots = true;
        //console.log("HI")
        if (previous_shots_time.length == 5) {
            previous_shots_time.shift();
        }
        previous_shots_time.push(time_since_last_shot);
    }

    // rolling average of player ship shot frequency
    total = 0;
    for (i=0; i < previous_shots_time.length; i += 1) {
        total += previous_shots_time[i];
    }
    average_frequency = total / previous_shots_time.length;



    //// All happens on the server
    //// ---------- AI Logic

    //// Determine whether ai_ship is able to shoot: true if no AI bullet is active and the shot_cooldown timer has expired

    var ai_shoots = false;
    var ai_bullet = ai_ship.bullets_group.getChildren()[0];
    var ai_bullet_position = [];

    // if (previous_shots.length == 5) {
    //    ai_ship.sprite.props.shot_cooldown = Math.min(...previous_shots, 55) - 5;
    // }

    // if (ai_bullet.active) {
    //     ai_bullet_position = [ai_bullet.body.x, ai_bullet.body.y];
    // }
    // else if (frame_number > (ai_ship.sprite.props.last_shot + ai_ship.sprite.props.shot_cooldown)) {
    //     ai_shoots = true;
    // }

    if (ai_bullet.active) {
        ai_bullet_position = [ai_bullet.body.x, ai_bullet.body.y];
    }
    //if (frame_number > ai_ship.sprite.props.last_shot + max_ai_frequency) {
    if (Date.now() > ai_ship.sprite.props.last_shot_time + max_ai_frequency) {
        ai_shoots = true;
    }



    var enemies_right_sprites = enemies_right.enemies_group.getChildren();
    var enemies_left_sprites = enemies_left.enemies_group.getChildren();

    
    var enemies_right_bullets = enemies_right.bullets_group.getChildren();
    var bullets_right_positions = []; // for logging purposes
    for (var i = 0; i < enemies_right_bullets.length; i++) {
        var current_bullet = enemies_right_bullets[i];
        
        if (current_bullet.active) {
            bullets_right_positions.push([current_bullet.x, current_bullet.y]);

        }
    }

    var enemies_left_bullets = enemies_left.bullets_group.getChildren();
    var bullets_left_positions = []; // for logging purposes
    for (var i = 0; i < enemies_left_bullets.length; i++) {
        var current_bullet = enemies_left_bullets[i];
        
        if (current_bullet.active) {
            bullets_left_positions.push([current_bullet.x, current_bullet.y]);

        }
    }


    // find the enemy that is closest to the ai player (on x-axis)
    var nearest_enemy = {x: 0, y: 0};
    var nearest_x_diff = this.sys.canvas.width;

    // ai-side enemies
    var enemies_right_positions = [];
    for (var i=0; i < enemies_right_sprites.length; i++) {
        var current_enemy = enemies_right_sprites[i];

        enemies_right_positions.push([current_enemy.x, current_enemy.y]);
        if (current_enemy.y > 540) {
            ai_over = true;
            //console.log("AI OVER")
        }

    }

    // player-side enemies
    var enemies_left_positions = [];
    for (var i=0; i < enemies_left_sprites.length; i++) {
        var current_enemy = enemies_left_sprites[i];

        enemies_left_positions.push([current_enemy.x, current_enemy.y]);
        if (current_enemy.y > 540) {
            player_over = true;
        }
    }
    
    var to_nearest_enemy = nearest_enemy.x - ai_ship.sprite.x

    // TODO: this seems like state, probably should send the actual state you care about though
    var log_frame = {
        frame_number: frame_number,                          //!< Number of the frame
        player_position: player_ship.sprite.x,               //!< Player's position
        player_lives: player_ship.sprite.props.lives,        //!< Player's lives
        player_score: player_ship.sprite.props.score,        //!< Player's score
        player_bullet_position: player_bullet_position,      //!< Positions of all of player's bullets

        ai_position: ai_ship.sprite.x,                       //!< AI Ship's position 
        ai_lives: ai_ship.sprite.props.lives,                //!< AI Ship's lives
        ai_score: ai_ship.sprite.props.score,                //!< AI Ship's score
        ai_bullet_position: ai_bullet_position,              //!< Positions of all of AI player's bullets

        enemies_left_positions: enemies_left_positions,      //!< Left side enemies' positions
        bullets_left_positions: bullets_left_positions,      //!< Positions of all left side enemies' bullets

        enemies_right_positions: enemies_right_positions,    //!< Right side enemies' positions
        bullets_right_positions: bullets_right_positions,    //!< Positions of all right side enemies' bullets

        nearest_enemy: nearest_enemy,
        //nearest_bullet: nearest_bullet,

        can_shoot: ai_shoots,

        player_last_shot_time: player_ship.sprite.props.last_shot_time, // frame when player last shot
        ai_last_shot_time: ai_ship.sprite.props.last_shot_time,         // frame when ai last shot
        player_last_shot_frame: player_ship.sprite.props.last_shot_frame, // frame when player last shot
        ai_last_shot_frame: ai_ship.sprite.props.last_shot_frame,         // frame when ai last shot
        player_avg_frequency: average_frequency,

        ai_pos_y: ai_ship.sprite.y,
        ai_ship_min_x: ai_ship.min_x,
        canvas_width: this.sys.canvas.width

    }

    // if (last_frame != frame_number){
    //     sockets.control.send(JSON.stringify(log_frame));
    //     last_frame = frame_number;
    //     console.log("SEND",frame_number)
    // }

    //player_ship.update(cursors.left.isDown, cursors.right.isDown, player_shoots);
    // REPLACES:
    //ai_ship.update(left_final, right_final, shoot_final);
    if (ai_ready == false && frame_number != 0) {
        //console.log("waiting: ", frame_number);
        //this.scene.pause();
        //game_paused = true;
        //console.log("PAUSE");
    } else if (ai_ready || frame_number == 0) {
        //console.log("** Updating player")
        player_ship.update(cursors.left.isDown, cursors.right.isDown, player_shoots);
        // enforce rules of the game
        var shoot = ai_commands.shoot && ai_shoots;
        var left = ai_commands.left && !ai_commands.right //&& !ai_shoots;
        var right = ai_commands.right && !ai_commands.left //&& !ai_shoots;
        //console.log("**Updating ai")
        ai_ship.update(left, right, shoot);
        ai_ready = false;
        // update the enemies
        enemies_left.update();
        enemies_right.update();
        //frames.push(log_frame);
        sockets.control.send(JSON.stringify(log_frame))
        //console.log("SEND", frame_number)
        //last_frame = frame_number;
    }
    frames.push(log_frame);
    frame_number += 1;
    //console.log("**Game running")

    // ---------- end AI logic

    

    // --- log this frame of the game ---

    // frames.push(log_frame);
    // frame_number += 1;
    if (enemies_left_sprites.length == 0) {
        player_over = true;
    }
    if (enemies_right_sprites.length == 0 && (mode == 3|| enemies_left_sprites.length == 0)) {
        ai_over = true;
    }

    // switch to game over screen
    if (player_over && ai_over) {
        game_log.push({player_id: player_id, date: date, round: rounds_played, mode: mode, events: events, frames: frames});
        // clearInterval(recording);
        //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
        this.scene.start('gameover_scene');
    }
}
