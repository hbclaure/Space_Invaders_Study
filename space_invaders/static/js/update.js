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
    else if (this.input.keyboard.checkDown(space_key, 500)) {
        player_shoots = true;
        if (previous_shots.length == 5) {
            previous_shots.shift();
        }
        previous_shots.push(frame_number - player_ship.sprite.props.last_shot);
    }


    //// All happens on the server
    //// ---------- AI Logic

    //// Determine whether ai_ship is able to shoot: true if no AI bullet is active and the shot_cooldown timer has expired

    var ai_shoots = false;
    var ai_bullet = ai_ship.bullets_group.getChildren()[0];
    var ai_bullet_position = [];

    //if (previous_shots.length == 5) {
    //    ai_ship.sprite.props.shot_cooldown = Math.min(...previous_shots, 55) - 5;
    //}

    if (ai_bullet.active) {
        ai_bullet_position = [ai_bullet.body.x, ai_bullet.body.y];
    }
    else if (frame_number > (ai_ship.sprite.props.last_shot + ai_ship.sprite.props.shot_cooldown)) {
        ai_shoots = true;
    }

    //var left_final = false;
    //var right_final = false;
    //var shoot_final = false;
    //var hit = false;

    var enemies_right_sprites = enemies_right.enemies_group.getChildren();
    var enemies_left_sprites = enemies_left.enemies_group.getChildren();

    // determine whether to attack left or right enemies
    var attack_left = false;
   
    if (mode == COOPERATIVE_EARLY) {
        if (enemies_right_sprites.length > 17) {
            attack_left = false;
        }
        else if (enemies_left_sprites.length > 12) {
            attack_left = true;
        }
        else if (enemies_right_sprites.length > 9) {
            attack_left = false;
        }
        else if (enemies_left_sprites.length > 0) {
            attack_left = true;
        }
        else {
            attack_left = false;
        }
    }
    else if (mode == COOPERATIVE_LATE) {
        attack_left = (enemies_right_sprites.length == 0) ? true : false;
    }
    

    // find the bullet that is closest to the AI player (on y-axis)
    var nearest_bullet = {x: 0, y: 0 };
    
    var enemies_right_bullets = enemies_right.bullets_group.getChildren();
    var bullets_right_positions = []; // for logging purposes
    for (var i = 0; i < enemies_right_bullets.length; i++) {
        var current_bullet = enemies_right_bullets[i];
        
        if (current_bullet.active) {
            bullets_right_positions.push([current_bullet.x, current_bullet.y]);

            if (ai_ship.sprite.x >= 400 && 
                current_bullet.y < ai_ship.sprite.y + 15 && current_bullet.y > nearest_bullet.y) {
                nearest_bullet.x = current_bullet.x;
                nearest_bullet.y = current_bullet.y;
            }
        }
    }

    var enemies_left_bullets = enemies_left.bullets_group.getChildren();
    var bullets_left_positions = []; // for logging purposes
    for (var i = 0; i < enemies_left_bullets.length; i++) {
        var current_bullet = enemies_left_bullets[i];
        
        if (current_bullet.active) {
            bullets_left_positions.push([current_bullet.x, current_bullet.y]);

            if (ai_ship.sprite.x < 400 && 
                current_bullet.y < ai_ship.sprite.y + 15 && current_bullet.y > nearest_bullet.y) {
                nearest_bullet.x = current_bullet.x;
                nearest_bullet.y = current_bullet.y;
            }
        }
    }


    // find the enemy that is closest to the ai player (on x-axis)
    var nearest_enemy = {x: 0, y: 0};
    var nearest_x_diff = this.sys.canvas.width;

    
    var enemies_right_positions = [];
    for (var i=0; i < enemies_right_sprites.length; i++) {
        var current_enemy = enemies_right_sprites[i];

        enemies_right_positions.push([current_enemy.x, current_enemy.y]);
        if (current_enemy.y > player_ship.sprite.y) {
            gameover = true;
        }

        var x_diff = Math.abs(current_enemy.x - ai_ship.sprite.x);

        if (!attack_left && x_diff < nearest_x_diff && !current_enemy.hit) {
            nearest_enemy.x = current_enemy.x;
            nearest_enemy.y = current_enemy.y;
            nearest_x_diff = x_diff;
        }
    }

    var enemies_left_positions = [];
    for (var i=0; i < enemies_left_sprites.length; i++) {
        var current_enemy = enemies_left_sprites[i];

        enemies_left_positions.push([current_enemy.x, current_enemy.y]);
        if (current_enemy.y > player_ship.sprite.y) {
            gameover = true;
        }

        var x_diff = Math.abs(current_enemy.x - ai_ship.sprite.x);

        if (attack_left && x_diff < nearest_x_diff && !current_enemy.hit) {
            nearest_enemy.x = current_enemy.x;
            nearest_enemy.y = current_enemy.y;
            nearest_x_diff = x_diff;
        }
    }
    //
    //// dodge logic: dodges the closest bullet when ai player is unable to fire
    //if (!ai_shoots) {
    //    if (nearest_bullet.x <= ai_ship.sprite.x + 30 && nearest_bullet.x >= ai_ship.sprite.x - 30) {
    //        hit = true;
    //    }

    //    if (hit && nearest_bullet.x >= this.sys.canvas.width - 75) {
    //        left_final = true;
    //    }
    //    else if (hit && nearest_bullet.x <= ai_ship.min_x + 55) {
    //        right_final = true;
    //    }
    //    else if (hit && nearest_bullet.x > ai_ship.sprite.x) {
    //        left_final = true;
    //    }
    //    else if (hit && nearest_bullet.x <= ai_ship.sprite.x) {
    //        right_final = true;
    //    }
    //}
    //// attack logic: move to the nearest enemy and shoot
    //else {
    //    if (nearest_x_diff <= 24) {
    //        shoot_final = true;
    //    }
    //    if (nearest_enemy.x < ai_ship.sprite.x) {
    //        // make sure it doesn't drive into bullets
    //        if (!(nearest_bullet.x < ai_ship.sprite.x && 
    //            nearest_bullet.y < ai_ship.sprite.y + 200 &&
    //            ai_ship.sprite.x - nearest_bullet.x <= 35)) {
    //            left_final = true;
    //        }
    //    }
    //    if (nearest_enemy.x > ai_ship.sprite.x) {
    //        if (!(nearest_bullet.x > ai_ship.sprite.x && 
    //            nearest_bullet.y < ai_ship.sprite.y + 200 && 
    //            nearest_bullet.x - ai_ship.sprite.x <= 35)) {
    //            right_final = true;
    //        }
    //    }
    //}

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
    }

    sockets.control.send(JSON.stringify(log_frame));

    // REPLACES:
    //ai_ship.update(left_final, right_final, shoot_final);
    // TODO: wait for the response from the server for the update (as opposed to asyc, as it is now)
    if (ai_ready) {
      player_ship.update(cursors.left.isDown, cursors.right.isDown, player_shoots);
      // enforce rules of the game
      var shoot = ai_commands.shoot && ai_shoots;
      var left = ai_commands.left && !ai_commands.right && !ai_shoots;
      var right = ai_commands.right && !ai_commands.left && !ai_shoots;
      ai_ship.update(left, right, shoot);
      ai_ready = false;
    }

    // ---------- end AI logic

    // update the enemies
    enemies_left.update();
    enemies_right.update();

    // --- log this frame of the game ---

    frames.push(log_frame);
    frame_number += 1;

    // switch to game over screen
    if (gameover || (enemies_left_sprites.length == 0 && enemies_right_sprites.length == 0)) {
        game_log.push({player_id: player_id, date: date, round: rounds_played, mode: mode, events: events, frames: frames});
        if (rounds_played == 0) {
            this.scene.start('intermediate_scene');
            rounds_played += 1;
            total_frames = frame_number;
        }
        else {
            this.scene.start('gameover_scene');
        }
    }
}
