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


    ai_ship = this.create_ship(0, "jordan", 1, this.sys.canvas.width / 2, 540, 5, "laser", 0, 30);
    enemies_left = this.create_enemies(6, 30, 50, "a", 5, 10, "enemylaser", min_x = 0, max_x = 350); 
    enemies_right = this.create_enemies(6, 450, 50, "b", 5, 10, "enemylaser", min_x = 420, max_x = 600);




 
        
    
     // --> enemies hit by Ai ship's bullets
     this.physics.add.overlap(enemies_right.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
         // destroy the enemy
         this.custom_sounds.enemy_explosion.play();
         // enemy.play(enemy.explote_anim, true);
         // enemy.on('animationcomplete', () => {
         // enemy.destroy();
         enemy.setVisible(false);
         

         // update the score
         if (enemy.is_hit == false) {
             bullet.body.x = this.sys.canvas.width;
             bullet.body.y = this.sys.canvas.height;
             bullet.setActive(false);
             // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);
 
             total_score += enemy.score;
            //  if (ai_supporting == 1) {
            //      p1_score_text.setText("ORANGE SCORE " + players[ai_supporting-1].sprite.props.score);
            //  } else if (ai_supporting == 2) {
            //      p2_score_text.setText("BLUE SCORE " + players[ai_supporting-1].sprite.props.score);
            //  }
             // p1_score_text.setText("SCORE " + total_score);
             events.push({frame: frame_number, killer: 'AI', killed: 'LEFT', type: 'SHOT'});
 
             enemy.is_hit = true;
             enemy.shot_frame = frame_number;
             num_enemies_shot = num_enemies_shot + 1;
             // enemy.hit = true;
         }
         // enemy.hit = true;
 
         enemy.time_hit = timer;
 
  
     });

     this.physics.add.overlap(enemies_left.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.enemy_explosion.play();
        // enemy.play(enemy.explote_anim, true);
        // enemy.on('animationcomplete', () => {
        // enemy.destroy();
        enemy.setVisible(false);
        

        // update the score
        if (enemy.is_hit == false) {
            bullet.body.x = this.sys.canvas.width;
            bullet.body.y = this.sys.canvas.height;
            bullet.setActive(false);
            // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);

            total_score += enemy.score;
            // if (ai_supporting == 1) {
            //     p1_score_text.setText("ORANGE SCORE " + players[ai_supporting-1].sprite.props.score);
            // } else if (ai_supporting == 2) {
            //     p2_score_text.setText("BLUE SCORE " + players[ai_supporting-1].sprite.props.score);
            // }
            // p1_score_text.setText("SCORE " + total_score);
            events.push({frame: frame_number, killer: 'AI', killed: 'LEFT', type: 'SHOT'});

            enemy.is_hit = true;
            enemy.shot_frame = frame_number;
            num_enemies_shot = num_enemies_shot + 1;
            // enemy.hit = true;
        }
        // enemy.hit = true;

        enemy.time_hit = timer;


    });


    this.physics.add.overlap(enemies_right.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.enemy_explosion.play();
        // enemy.play(enemy.explote_anim, true);
        // enemy.on('animationcomplete', () => {
        // enemy.destroy();
        enemy.setVisible(false);
        

        // update the score
        if (enemy.is_hit == false) {
            bullet.body.x = this.sys.canvas.width;
            bullet.body.y = this.sys.canvas.height;
            bullet.setActive(false);
            // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);

            total_score += enemy.score;
            // if (ai_supporting == 1) {
            //     p1_score_text.setText("ORANGE SCORE " + players[ai_supporting-1].sprite.props.score);
            // } else if (ai_supporting == 2) {
            //     p2_score_text.setText("BLUE SCORE " + players[ai_supporting-1].sprite.props.score);
            // }
            // p1_score_text.setText("SCORE " + total_score);
            events.push({frame: frame_number, killer: 'AI', killed: 'LEFT', type: 'SHOT'});

            enemy.is_hit = true;
            enemy.shot_frame = frame_number;
            num_enemies_shot = num_enemies_shot + 1;
            // enemy.hit = true;
        }
        // enemy.hit = true;

        enemy.time_hit = timer;


    });

    gameover = false;


    function get_time(timer) {
        var minutes = Math.floor(timer / 60);
        var seconds;
        if (timer - (minutes * 60) < 10) {
            // console.log("less than 10");
            remaining = timer - (minutes * 60);
            // console.log("remaining" + remaining)
            seconds = "0" + remaining.toString();
        } else {
            seconds = timer - (minutes * 60);
        }
        // console.log("time" + minutes + seconds)
        var time_string = minutes + ":" + seconds;
        return time_string;
    }

    time = get_time(timer);
    var timer_text = this.add.bitmapText(this.sys.canvas.width * 0.75, 3, 'PressStart2P_White', 'TIMER ' + time, 20);
    timer_interval = setInterval(function () {
        timer = timer - 1;
        time = get_time(timer);
        timer_text.setText('TIMER ' + time)
        // if (timer == 0) {
        //     window.clearInterval(timer_interval);
        // }
    }, 1000)

    console.log('hi')


   // log state

   



}


function update_practice_scene() {
    console.log('start update')



    // //// --- AI Logic

    // }
    //// Determine whether ai_ship is able to shoot: true if no AI bullet is active and the shot_cooldown timer has expired
    var ai_shoots = false;

    if (Date.now() - ai_ship.sprite.props.last_shot_time >= max_ai_frequency) {
        ai_shoots = true;
    }

    ai_received_action = {left: ai_commands.left, right: ai_commands.right, shoot: ai_commands.shoot, support: ai_commands.support};
 

    if (ai_ready == false) {
        ai_actual_action = {left: false, right: false, shoot: false, support: 0};
        
        
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




        ai_ship.update(left, right, shoot, support)

        ai_actual_action = {left: left, right: right, shoot: shoot, support: support};

        enemies_left.update();
        enemies_right.update();
        frame_sent = true;
    }

    // ai bullets
    var ai_bullets = ai_ship.bullets_group.getChildren();
    var ai_bullets_positions = []; // for logging purposes
    for (var i = 0; i < ai_bullets.length; i++) {
        var current_bullet = ai_bullets[i];
        
        if (current_bullet.active) {
            ai_bullets_positions.push([current_bullet.x, current_bullet.y]);

        }        if (current_bullet.y < 50) {
            current_bullet.setActive(false);
            current_bullet.body.x = this.sys.canvas.width;
            current_bullet.body.y = this.sys.canvas.height;
        }
        
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
    // --- log state
    var log_frame = {
        frame_number: frame_number,                          //!< Number of the frame
        timestamp: Date.now(),

  
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

   
        ai_last_shot_time: ai_ship.sprite.props.last_shot_time,         // frame when ai last shot
    
        ai_last_shot_frame: ai_ship.sprite.props.last_shot_frame,         // frame when ai last shot

        frame_sent: frame_sent,
        ai_actual_action: ai_actual_action,
        ai_received_action: ai_received_action,

        // for error signaling

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
    console.log('timer',timer)
    if (timer == 0) {
        events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});

        for (var i = 0; i < 4; i++) {

            ai_ship.sprite.lives[3].setVisible(false);
            ai_ship.sprite.lives[2].setVisible(false);
            ai_ship.sprite.lives[1].setVisible(false);
            ai_ship.sprite.lives[0].setVisible(false);
        }
        ai_ship.sprite.props.dead = true;
        ai_ship.sprite.props.lives = 0;
        ai_over = true;
    }
    // const endFunc = () => {
    //     console.log("end function");
    //     this.scene.start('gameover_scene'); 
    // }
    if ( ai_over) {
        console.log("game over")
        window.clearInterval(timer_interval);
        game_log = {player_id: player_id, date: date, mode: mode, events: events, frames: frames};
        // clearInterval(recording);
        //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
        this.scene.start('practice_over_scene');
    }






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



    var gameover_text = this.add.bitmapText(400, 125, 'PressStart2P_Orange', 'ROUND ENDED', 50).setOrigin(0.5);
    // var player_text = this.add.bitmapText(400, 250, 'PressStart2P_Purple', 'Player Final Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    // var font_type = (mode == UNCOOPERATIVE) ? 'PressStart2P_Orange' : 'PressStart2P_Gray';
    // var ai_text = this.add.bitmapText(400, 350, font_type, 'AI Final Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var font_type = 'PressStart2P_Green';
    // var final_score_text = this.add.bitmapText(400, 300, font_type, 'Final Score: ' + total_score, 30).setOrigin(0.5).setCenterAlign();
    var cc_text = this.add.bitmapText(400, 450, 'PressStart2P_White', 'Press B on the controller to continue', 20).setOrigin(0.5).setCenterAlign();
    // var cc = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Loading...', 20).setOrigin(0.5).setCenterAlign();
    // log this game
    //sockets.log.onmessage = function(event) {
    // sockets.control.onmessage = function(event) {
    //     // if message == "saved", then do this; otherwise do nothing
    //     console.log('message received');
    //     if(event.data=="saved"){
    //         cc.destroy();
    //         practice_over_scene.add.bitmapText(400, 500, 'PressStart2P_Green', completion_code, 40).setOrigin(0.5).setCenterAlign();
    //     }  
    // }
    sockets.control.send(JSON.stringify(game_log));
    console.log(completion_code);
    //sockets.control.send(JSON.stringify({player_id: player_id, date: date, events: events}))
}


    // // ------ get state information
    // // player bullets


    // // log state
    // var log_frame = {
    //     frame_number: frame_number,                          //!< Number of the frame
    //     timestamp: Date.now(),
    //     ai_position: ai_ship.sprite.x,                       //!< AI Ship's position 
    //     ai_lives: ai_ship.sprite.props.lives,                //!< AI Ship's lives
    //     ai_score: ai_ship.sprite.props.score,                //!< AI Ship's score
    //     ai_bullets_positions: ai_bullets_positions,              //!< Positions of all of AI player's bullets
    //     ai_supporting: ai_supporting,
    //     enemies_left_positions: enemies_left_positions,      //!< Left side enemies' positions
    //     bullets_left_positions: bullets_left_positions,      //!< Positions of all left side enemies' bullets
    //     enemies_left_shot: enemies_left_shot,
    //     num_enemies_shot: num_enemies_shot,

    //     enemies_right_positions: enemies_right_positions,    //!< Right side enemies' positions
    //     bullets_right_positions: bullets_right_positions,    //!< Positions of all right side enemies' bullets
    //     ai_last_shot_time: ai_ship.sprite.props.last_shot_time,         // frame when ai last shot
    //     frame_sent: frame_sent,
    //     ai_actual_action: ai_actual_action,
    //     ai_received_action: ai_received_action,


    //     //enemies_middle_positions: enemies_middle_positions,
    //     //bullets_middle_posisionts: bullets_middle_positions,

    //     can_shoot: ai_shoots,



    //     // for error signaling
    //     signal_down: signal_down,
    //     signal_up: signal_up,
    //     // if player pressed but it is during the delay
    //     tried_signal_down: tried_signal_down,
    //     tried_signal_up: tried_signal_up
    // }
    // frames.push(log_frame);

    // if (frame_sent){
    //     sockets.control.send(JSON.stringify(log_frame));
    //     ai_ready = false;
    // }
    
    // // --- send state if server was ready
    // // if (frame_sent){
    // //     sockets.control.send(JSON.stringify(log_frame));
    // //     ai_ready = false;
    // // }

    // // --- log frame of game
    // // frames.push(log_frame);
    // frame_number += 1;

    // signal_down = false;
    // signal_up = false;
    // tried_signal_down = false;
    // tried_signal_up = false;

    // game_log = { date: date, frames: frames};
    
    // // if (instruction_num == 6 && this.input.keyboard.checkDown(q_key, 500)) {
    // //     this.scene.start('practice_over_scene');
    // }    
