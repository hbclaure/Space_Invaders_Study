/**
 * Update the state of the game
 */

var last_msg_frame = -500;
var signal_up = false;
var signal_down = false;
var tried_signal_down = false;
var tried_signal_up = false;
var feedback_enabled = true;
//TO DO: STORAGE OF ACTIONS FOR NEW AI HAS TO BE FIXED- double check
//make sure its not doing the action for both players if one is controlling the other. 

    //s = 1 Player 2 which is the Shutter Robot 
    //s = 0 Player 1 or participant which is controled by keyboard or controller 

function update ()
{
    // var all_player_bullets_positions = [];      // for logging purposes
    // var all_bullets_positions = [];     // for logging purposes

    


    var player_tried_to_shoot = false;
    var player_shoots = false;
    var time_since_last_shot = Date.now() - players[0].sprite.props.last_shot_time;


    // var shoot_button;
    if (gamepad1 == true) {
        if (p1xBoxA == true) {
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
    } 
    
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

    //// --- AI Logic

    //// Determine whether ai_ship is able to shoot: true if no AI bullet is active and the shot_cooldown timer has expired
    var ai_shoots = false;
    var ai_shoots_player = false;

    if (Date.now() - ai_ship.sprite.props.last_shot_time >= max_ai_frequency) {
        ai_shoots = true;
        console.log('ai ship', ai_shoots)
    }

    if (Date.now() - players[1].sprite.props.last_shot_time >= max_ai_frequency) {
        ai_shoots_player = true;
        console.log(ai_shoots_player)

    }

    //// --- keep track of actions

    //s = 1 Player 2 which is the Shutter Robot 
    //s = 0 Player 1 or participant which is controled by keyboard or controller 
    
    if (gamepad1 == true ) {
        //If using Gamepad for controlling Participant Ship 
        player_left_participant = p1xBoxLeft;
        player_right_participant = p1xBoxRight;
        player_up_participant = false;
        player_down_participant = false;
    }
    else {
        //if using keyboard
        player_left_participant = cursors.left.isDown;;
        player_right_participant = cursors.right.isDown;;
        player_up_participant = cursors.up.isDown;
        player_down_participant = cursors.down.isDown;
    }

        //Shutter AI commands
        player_left = ai_commands.player_left;
        player_right = ai_commands.player_right;
        player_up = false;
        player_down= false;

    //check shooting
    player_action = {left: player_left_participant, right: player_right_participant, shoot: player_shoots, tried_to_shoot: player_tried_to_shoot};
    
    ai_received_action = {left: ai_commands.left, right: ai_commands.right, shoot: ai_commands.shoot, support: ai_commands.support};
    ai_player_received_action = {left_player: player_left, right_player: player_right, shoot_player: ai_commands.shoot_player, support_player: ai_commands.support_player};
    
    if (ai_ready == false) {
        ai_actual_action = {left: false, right: false, shoot: false, support: 0};
        shutter_player_actual_action = {left_player: false, right_player: false, shoot_player: false};
        
        
        if (frame_number == 0){
            frame_sent = true;
        } else{
            frame_sent = false;
        }
    } else if (ai_ready) {

        // update ai agent, enforce rules of the game
        var shoot = ai_commands.shoot && ai_shoots;
        var left = ai_commands.left && !ai_commands.right //&& !ai_shoots;
        var right = ai_commands.right && !ai_commands.left //&& !ai_shoots;
        var support = ai_commands.support;

        //Update Shutter AI agent
        var shutter_shoot = ai_commands.player_shoot && ai_shoots_player;
        var shutter_left = ai_commands.player_left && !ai_commands.player_right //&& !ai_shoots;
        var shutter_right = ai_commands.player_right && !ai_commands.player_left //&& !ai_shoots;



        // update player -Shutter AI partner
        //WILL NEED TO CHANGE TO ABOVE (SHOOT PLAYER) ONCE IT CAN SHOOT
        players[1].update(shutter_left, shutter_right, shutter_shoot)
        //console.log('shutter Left', player_left, 'shutter right',player_right);

        //AI Ship- Nao
        ai_ship.update(left, right, shoot, support);

        //Participant Ship
        players[0].update(player_left_participant,player_right_participant,player_shoots);
        //console.log('parti left',player_left_participant,'parti right', player_right_participant )

        //will need to be updated once shooting is enabled
        ai_actual_action = {left: left, right: right, shoot: shoot, support: support};
        shutter_player_actual_action = {left_player: shutter_left, right_player: shutter_right, shoot_player: shutter_shoot};

        enemies_left.update();
        enemies_right.update();
        
        frame_sent = true;
    }

        

        
    //END




    // ------ get state information
    // player1 bullets
    var player_bullets = players[0].bullets_group.getChildren();
    var player1_bullets_positions = []; // for logging purposes
    for (var i = 0; i < player_bullets.length; i++) {
        var current_bullet = player_bullets[i];
        
        if (current_bullet.active) {
            player1_bullets_positions.push([current_bullet.x, current_bullet.y]);

        }
    }
    // player2 bullets
    var player_bullets = players[1].bullets_group.getChildren();
    var player2_bullets_positions = []; // for logging purposes
    for (var i = 0; i < player_bullets.length; i++) {
        var current_bullet = player_bullets[i];
        
        if (current_bullet.active) {
            player2_bullets_positions.push([current_bullet.x, current_bullet.y]);

        }
    }
    // all_player_bullets_positions.push(player_bullets_positions);

    // ai bullets
    var ai_bullets = ai_ship.bullets_group.getChildren();
    var ai_bullets_positions = []; // for logging purposes
    for (var i = 0; i < ai_bullets.length; i++) {
        var current_bullet = ai_bullets[i];
        
        if (current_bullet.active) {
            ai_bullets_positions.push([current_bullet.x, current_bullet.y]);

        }
    }

    // enemies
    var enemies_right_sprites = enemies_right.enemies_group.getChildren();
    var enemies_left_sprites = enemies_left.enemies_group.getChildren();
    //console.log('this right sprites',enemies_right_sprites, 'left sprites', enemies_left_sprites)
    //var enemies_middle_sprites = enemies_middle.enemies_group.getChildren();

    // right side bullets
    var enemies_right_bullets = enemies_right.bullets_group.getChildren();
    var bullets_right_positions = []; // for logging purposes
    for (var i = 0; i < enemies_right_bullets.length; i++) {
        var current_bullet = enemies_right_bullets[i];
        
        if (current_bullet.active) {
            bullets_right_positions.push([current_bullet.x, current_bullet.y]);

        }
    }
    
    // left side bullets
    var enemies_left_bullets = enemies_left.bullets_group.getChildren();
    var bullets_left_positions = []; // for logging purposes
    for (var i = 0; i < enemies_left_bullets.length; i++) {
        var current_bullet = enemies_left_bullets[i];
        
        if (current_bullet.active) {
            bullets_left_positions.push([current_bullet.x, current_bullet.y]);

        }
    }

    // ai-side (right) enemies
    var enemies_right_positions = [];
    for (var i=0; i < enemies_right_sprites.length; i++) {
        var current_enemy = enemies_right_sprites[i];

        enemies_right_positions.push([current_enemy.x, current_enemy.y]);
        // end game if enemies reach bottom
        if (current_enemy.y > 540) {
            ai_over = true;
            player_over = true;
        }

    }
    
    // player-side (left) enemies
    var enemies_left_positions = [];
    for (var i=0; i < enemies_left_sprites.length; i++) {
        var current_enemy = enemies_left_sprites[i];

        enemies_left_positions.push([current_enemy.x, current_enemy.y]);
        // end game if enemies reach bottom
        if (current_enemy.y > 540) {
            player_over = true;
            ai_over = true;
        }
    }
    var enemies_left_shot = []
    for (var i = 0; i < enemies_left_sprites.length; i++) {
        var current_enemy = enemies_left_sprites[i];
        enemies_left_shot.push(current_enemy.is_hit);
    }

    // Set Winning Icon visible or Tied message

    // if (players[0].sprite.props.score< players[1].sprite.props.score){
    //     //if shutter  is winning
    //     console.log('Shutter winning')
    //     icon.setVisible(true); 

        
    //     icon.x = this.sys.canvas.width / 2 + this.sys.canvas.width / 4 + 100;
    //     players[0].sprite.props.message.visible = false;
    //     players[1].sprite.props.message.visible = false;




    // }
    // else if (players[0].sprite.props.score> players[1].sprite.props.score) {
    //     //if human is winning
    //     icon.setVisible(true); 

    //     icon.x =  this.sys.canvas.width / 4 - 100;
    //     players[0].sprite.props.message.visible = false;
    //     players[1].sprite.props.message.visible = false;

    // }
    // else {
    //     //hide icon
    //     icon.setVisible(false); 
    //     //if scores are tied
    //     players[0].sprite.props.message.text = "Tied Scores!"
    //     players[0].sprite.props.message.visible = true;
    //     players[0].sprite.props.message.align = 1;

    //     players[1].sprite.props.message.text = "Tied Scores!"
    //     players[1].sprite.props.message.visible = true;
    //     players[1].sprite.props.message.align = 1;

    // }

    const scoreDifference = players[1].sprite.props.score - players[0].sprite.props.score;
    const movementSpeed =.05;
    // Calculate the target x position of the icon based on the score difference
    const targetX = Phaser.Math.Clamp(
        this.sys.canvas.width / 2 + (scoreDifference * 50), // Adjust the factor (50) to control the movement speed
        this.sys.canvas.width / 4 - 100, // Left boundary
        this.sys.canvas.width / 2 + this.sys.canvas.width / 4 + 100 // Right boundary
    );

    // Move the icon towards the target x position
    const diffX = targetX - icon.x;
    icon.x += diffX * movementSpeed;


    // --- log state
    var log_frame = {
        frame_number: frame_number,                          //!< Number of the frame
        timestamp: Date.now(),

        player1_position: players[0].sprite.x,               //!< Player's position
        player2_position: players[1].sprite.x, 
        player1_lives: players[0].sprite.props.lives,        //!< Player's lives
        player2_lives: players[1].sprite.props.lives,
        player1_score: players[0].sprite.props.score,        //!< Player's score
        player2_score: players[1].sprite.props.score,
        player1_bullets_positions: player1_bullets_positions,   //!< Positions of all of player's bullets
        player2_bullets_positions: player2_bullets_positions,

        ai_position: ai_ship.sprite.x,                       //!< AI Ship's position 
        ai_lives: ai_ship.sprite.props.lives,                //!< AI Ship's lives
        ai_score: ai_ship.sprite.props.score,                //!< AI Ship's score
        ai_bullets_positions: ai_bullets_positions,              //!< Positions of all of AI player's bullets
        ai_supporting: ai_supporting,

        enemies_left_positions: enemies_left_positions,      //!< Left side enemies' positions
        bullets_left_positions: bullets_left_positions,      //!< Positions of all left side enemies' bullets
        enemies_left_shot: enemies_left_shot,
        // enemies_mult_5: change_team,
        num_enemies_shot: num_enemies_shot,

        enemies_right_positions: enemies_right_positions,    //!< Right side enemies' positions
        bullets_right_positions: bullets_right_positions,    //!< Positions of all right side enemies' bullets

        //enemies_middle_positions: enemies_middle_positions,
        //bullets_middle_posisionts: bullets_middle_positions,

        can_shoot: ai_shoots,
        can_shoot_shutter: ai_shoots_player,

        player1_last_shot_time: players[0].sprite.props.last_shot_time, // frame when player last shot
        player2_last_shot_time: players[1].sprite.props.last_shot_time,
        ai_last_shot_time: ai_ship.sprite.props.last_shot_time,         // frame when ai last shot
        player1_last_shot_frame: players[0].sprite.props.last_shot_frame, // frame when player last shot
        player2_last_shot_frame: players[1].sprite.props.last_shot_frame,
        ai_last_shot_frame: ai_ship.sprite.props.last_shot_frame,         // frame when ai last shot
        player_avg_frequency: average_frequency,

        frame_sent: frame_sent,
        player_action: player_action,
        ai_actual_action: ai_actual_action,
        shutter_player_actual_action:shutter_player_actual_action,
        ai_player_received_action:ai_player_received_action,
        ai_received_action: ai_received_action,

        // for error signaling
        signal_down: signal_down,
        signal_up: signal_up,
        // if player pressed but it is during the delay
        tried_signal_down: tried_signal_down,
        tried_signal_up: tried_signal_up,
        timer:timer
    }
    
    // --- send state if server was ready
    if (frame_sent){
        sockets.control.send(JSON.stringify(log_frame));
        ai_ready = false;
    }

    // --- log frame of game
    frames.push(log_frame);
    frame_number += 1;

    signal_down = false;
    signal_up = false;
    tried_signal_down = false;
    tried_signal_up = false;

    // --- check game over conditions ---
    // if (enemies_left_sprites.length == 0 && enemies_middle_sprites.length == 0) {
    //     player_over = true;
    // }
    // if (enemies_right_sprites.length == 0 && (mode == 3|| enemies_left_sprites.length == 0)) {
    //     ai_over = true;
    // }

    // switch to game over screen
    // switch to game over screen
    if (timer == 0) {
        events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'SHOT'});
        events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
        players[0].sprite.props.dead = true;
        players[0].sprite.props.lives = 0;
        players[1].sprite.props.dead = true;
        players[1].sprite.props.lives = 0;
        for (var i = 0; i < 4; i++) {
            players[0].sprite.lives[3].setVisible(false);
            players[0].sprite.lives[2].setVisible(false);
            players[0].sprite.lives[1].setVisible(false);
            players[0].sprite.lives[0].setVisible(false);
            players[1].sprite.lives[3].setVisible(false);
            players[1].sprite.lives[2].setVisible(false);
            players[1].sprite.lives[1].setVisible(false);
            players[1].sprite.lives[0].setVisible(false);
            ai_ship.sprite.lives[3].setVisible(false);
            ai_ship.sprite.lives[2].setVisible(false);
            ai_ship.sprite.lives[1].setVisible(false);
            ai_ship.sprite.lives[0].setVisible(false);
        }
        ai_ship.sprite.props.dead = true;
        ai_ship.sprite.props.lives = 0;
        player_over = true;
        ai_over = true;console.log
    }
    // const endFunc = () => {
    //     console.log("end function");
    //     this.scene.start('gameover_scene'); 
    // }
    if (player_over && ai_over) {
        window.clearInterval(timer_interval);
        game_log = {player_id: player_id, date: date, mode: mode, events: events, frames: frames};
        // clearInterval(recording);
        //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
        this.scene.start('gameover_scene');
    }


}
