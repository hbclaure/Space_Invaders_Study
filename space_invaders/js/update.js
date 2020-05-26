/**
 * Update the state of the game
 */
function update ()
{
    // --------- update the ship

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

    // manual control of ai update

    // ---------- AI shooting logic, ai ship shoots whenever it can

    var ai_shoots = false;
    var ai_bullet = ai_ship.bullets_group.getChildren()[0];
    var ai_bullet_position = [];


    if (ai_bullet.active) {
        ai_bullet_position = [ai_bullet.body.x, ai_bullet.body.y];
    }
    else {
        ai_shoots = true;
    }
    

    // ---------- start of AI movement logic

    left_final = false;
    right_final = false;

    var left_enemy = 800;
    var right_enemy = 0;

    move_left = move_right = true;
    hit = false;

    // --> checking where the enemies are
    var enemies_right_sprites = enemies_right.enemies_group.getChildren();
    var enemies_right_positions = [];
    for (var i=0; i < enemies_right_sprites.length; i++) {
        if (enemies_right_sprites[i].body.x > right_enemy) {
            right_enemy = enemies_right_sprites[i].body.x;
        }
        if (enemies_right_sprites[i].body.x < left_enemy) {
            left_enemy = enemies_right_sprites[i].body.x;
        }
        enemies_right_positions.push([enemies_right_sprites[i].body.x, enemies_right_sprites[i].body.y]);
        // if enemy has reached end of screen, game over
        if (enemies_right_sprites[i].body.y > player_ship.sprite.y) {
            gameover = true;
        }
    }

    var enemies_left_sprites = enemies_left.enemies_group.getChildren();
    var enemies_left_positions = [];
    for (var i=0; i < enemies_left_sprites.length; i++) {
        //don't consider enemies on left side if not cooperative agent
        if (mode == 0) {
            if (enemies_left_sprites[i].body.x > right_enemy) {
                right_enemy = enemies_left_sprites[i].body.x;
            }
            if (enemies_left_sprites[i].body.x < left_enemy) {
                left_enemy = enemies_left_sprites[i].body.x;
            }
        }
        enemies_left_positions.push([enemies_left_sprites[i].body.x, enemies_left_sprites[i].body.y]);
        if (enemies_left_sprites[i].body.y > player_ship.sprite.y) {
            gameover = true;
        }
    }
        

    // --> checking where the bullets are/if a bullet is about to hit the ai ship
    var bullets_right = enemies_right.bullets_group.getChildren();
    var bullets_right_positions = [];
    for(var i=0; i < bullets_right.length; i++){
        if (bullets_right[i].body.y < 100 || bullets_right[i].visible == false) {
            continue;
        }
        var x_diff = bullets_right[i].body.x - ai_ship.sprite.x;
        if (x_diff > -30 && x_diff < -1) {
            move_left = false;
        }
        if (x_diff >= -1 && x_diff <= 50) {
            hit = true;
        }
        if (x_diff > 50 && x_diff < 80) {
            move_right = false;
        }
        if (bullets_right[i].active) {
            bullets_right_positions.push([bullets_right[i].body.x, bullets_right[i].body.y]);
        }
    }

    var bullets_left = enemies_left.bullets_group.getChildren();
    var bullets_left_positions = [];          
    for(var i=0; i < bullets_left.length; i++){
        //don't consider bullets on left side if not cooperative agent
        if (mode == 0) {
            if (bullets_left[i].body.y < 100 || bullets_left[i].visible == false) {
                continue;
            }
            var diff = bullets_left[i].body.x - ai_ship.sprite.x;

            if (diff > -30 && diff < -1) {
                move_left = false;
            }
            if (diff >= -1 && diff <= 50) {
                hit = true;
            }
            if (diff > 50 && diff < 80) {
                move_right = false;
            }
        }
        if (bullets_left[i].active) {
            bullets_left_positions.push([bullets_left[i].body.x, bullets_left[i].body.y]);
        }
    }

    // debugging text
    if (ai_ship.sprite.props.dead == false && debug_text) {
        if (!move_left) {
            console.log("bullet on left");
        }
        if (hit) {
            console.log("bullet incoming");
        }   
        if (!move_right) {
            console.log("bullet on right");
        }
    }

    // --> deciding which direction to move
    if (move_left && move_right && hit && ai_ship.sprite.x < ai_ship.min_x + 10) {
        right_final = true;
    }
    else if (move_left && move_right && hit && ai_ship.sprite.x > this.sys.canvas_width - 60) {
        left_final = true;
    }
    else if (move_left && hit && ai_ship.sprite.x > ai_ship.min_x + 10) {
        left_final = true;
    }
    else if (move_right && hit && ai_ship.sprite.x < this.sys.canvas_width - 60) {
        right_final = true;
    }
    else if (ai_ship.sprite.x > right_enemy && move_left && ai_ship.sprite.x > ai_ship.min_x) {
        left_final = true;
    }
    else if ((ai_ship.sprite.x < right_enemy - 10 && move_right) || (ai_ship.sprite.x < this.sys.canvas_width - 60 && move_right)) {
        right_final = true;
    }


    // more debugging text
    if (ai_ship.sprite.props.dead == false && debug_text) {
        console.log("left: " + move_left + ". right: " + move_right + ", hit: " + hit + ", right enemy: " + right_enemy + ", left enemy: " + left_enemy);
        console.log("x: " + ai_ship.sprite.x);
        console.log("left: " + left_final + ", right: " + right_final);
    }

    ai_ship.update(left_final, right_final, ai_shoots);

    // ---------- end AI logic

    // update the enemies
    enemies_left.update();
    enemies_right.update();

    // --- log this frame of the game ---

    var log_frame = {
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

    game_log.push(log_frame);

    // switch to game over screen
    if (gameover) {
        console.log({id: game_id, log: game_log});
        this.scene.switch('gameover_scene');
    }

    // switch to victory screen
    if (enemies_left_sprites.length == 0 && enemies_right_sprites.length == 0) {
        console.log({id: game_id, log: game_log});
        this.scene.switch('victory_scene');
    }
}