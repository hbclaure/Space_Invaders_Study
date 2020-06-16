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
        game_log.push({player_id: player_id, date: date, round: 0, mode: mode, events, events, frames: frames});
        var xhr = new XMLHttpRequest();
        xhr.open('POST', '/log', true);
        xhr.setRequestHeader('Content-Type', 'application/json');
        xhr.send(JSON.stringify(game_log));
        this.scene.start('gameover_scene_practice');
    }
}


