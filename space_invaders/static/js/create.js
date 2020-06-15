// --- helper functions ---

/**
 * Create world objects and collider functions
 */
function create ()
{	
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

    player_ship = this.create_ship("ship", 0, this.sys.canvas.width / 4, 540);

    //setting the left wall for the AI depending on what kind of agent it is
    var minX = 0;
    if (mode == UNCOOPERATIVE) {
        minX = 425;
        ai_ship = this.create_ship("avery", 1, this.sys.canvas.width / 4 + 400, 540, 5, "laser", minX);
    }
    else  {
        ai_ship = this.create_ship("jordan", 1, this.sys.canvas.width / 4 + 400, 540, 5, "laser", minX);
    }

    //creating the enemies on the left and right
    var enemy_rows = 5;
    enemies_left = this.create_enemies(5, 30, 0, "a");
    enemies_right = this.create_enemies(5, 430, 0, "b", 5, 10, "enemylaser", min_x = 410, max_x = 800);

    // --> COLLIDERS <--
    // --> enemies hit by player_ship bullets 
    this.physics.add.collider(enemies_left.enemies_group, player_ship.bullets_group, (enemy, bullet) => {
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
        }
        enemy.hit = true;
        events.push({frame: frame_number, killer: 'PLAYER', killed: 'LEFT'});
    });
	this.physics.add.collider(enemies_right.enemies_group, player_ship.bullets_group, (enemy, bullet) => {
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
    	   ai_ship.sprite.props.score += enemy.score;
    	   ai_ship.sprite.props.scoreText.setText("SCORE " + ai_ship.sprite.props.score);
        }
        enemy.hit = true;
        events.push({frame: frame_number, killer: 'PLAYER', killed: 'RIGHT'});
    });

    // --> enemies hit by Ai ship's bullets
    this.physics.add.collider(enemies_left.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
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
        }
        enemy.hit = true;
        events.push({frame: frame_number, killer: 'AI', killed: 'LEFT'});
    });
	this.physics.add.collider(enemies_right.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
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
            ai_ship.sprite.props.score += enemy.score;
            ai_ship.sprite.props.scoreText.setText("SCORE " + ai_ship.sprite.props.score);
        }
        enemy.hit = true;
        events.push({frame: frame_number, killer: 'AI', killed: 'RIGHT'});
    });

    // --> enemies bullets hit ships bullets
    this.physics.add.collider(enemies_left.bullets_group, player_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
        enemy_bullet.setActive(false);
        ship_bullet.setActive(false);
    });
    this.physics.add.collider(enemies_right.bullets_group, player_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
        enemy_bullet.setActive(false);
        ship_bullet.setActive(false);
    });

    // --> enemies bullets hit AI ship's bullets
    this.physics.add.collider(enemies_left.bullets_group, ai_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
        enemy_bullet.setActive(false);
        ship_bullet.setActive(false);
    });
    this.physics.add.collider(enemies_right.bullets_group, ai_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
        enemy_bullet.setActive(false);
        ship_bullet.setActive(false);
    });

    // --> enemies bullets hit player_ship
    this.physics.add.collider(player_ship.sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
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
                ship_sprite.x = this.sys.canvas.width / 2;
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
    this.physics.add.collider(player_ship.sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        if (ship_sprite.props.invincible) { }
        // kill the player. The change in behavior takes place within the update function of the ship
        else if (ship_sprite.props.lives >= 1) {
            events.push({frame: frame_number, killer: 'RIGHT', killed: 'PLAYER'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                // give the player 50 frames of invincibility
                ship_sprite.props.invincible = true;
                ship_sprite.props.invincibility_timer = 50;
            });
        }
        else {
            events.push({frame: frame_number, killer: 'RIGHT', killed: 'PLAYER'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
            gameover = true;
        }
    });

    // --> enemies bullets hit ai_ship
    this.physics.add.collider(ai_ship.sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        if (ship_sprite.props.invincible) { }
        // kill the player. The change in behavior takes place within the update function of the ship
        else if (ship_sprite.props.lives >= 1) {
            events.push({frame: frame_number, killer: 'RIGHT', killed: 'AI'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2 + 25;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                // give the player 50 frames of invincibility
                ship_sprite.props.invincible = true;
                ship_sprite.props.invincibility_timer = 50;
            });
        }
        else {
            events.push({frame: frame_number, killer: 'RIGHT', killed: 'AI'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                ship_sprite.props.lives -= 1;
                ship_sprite.props.dead = true;
            });
        }
    });
    this.physics.add.collider(ai_ship.sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        if (ship_sprite.props.invincible) { }
        // kill the player. The change in behavior takes place within the update function of the ship
        else if (ship_sprite.props.lives >= 1) {
            events.push({frame: frame_number, killer: 'LEFT', killed: 'AI'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2 + 25;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                // give the player 50 frames of invincibility
                ship_sprite.props.invincible = true;
                ship_sprite.props.invincibility_timer = 50;
            });
        }
        else {
            events.push({frame: frame_number, killer: 'LEFT', killed: 'AI'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                ship_sprite.props.lives -= 1;
                ship_sprite.props.dead = true;
            });
        }
    });


    // --> enemies hit player_ship
    this.physics.add.collider(player_ship.sprite, enemies_left.enemies_group, (ship_sprite, enemy) => {
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
                ship_sprite.x = this.sys.canvas.width / 2;
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
    this.physics.add.collider(player_ship.sprite, enemies_right.enemies_group, (ship_sprite, enemy) => {
        // play sound
        this.custom_sounds.player_explosion.play();
        this.custom_sounds.enemy_explosion.play();
        // update the score
        ai_ship.sprite.props.score += enemy.score;
        ai_ship.sprite.props.scoreText.setText("SCORE " + ai_ship.sprite.props.score);
        // kill the player and the enemy. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives >= 1) {
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            enemy.play(enemy.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                enemy.destroy()
            });
        }
        else {
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
            gameover = true;
        }
    });

    // --> enemies hit ai_ship
    this.physics.add.collider(ai_ship.sprite, enemies_left.enemies_group, (ship_sprite, enemy) => {
        // play sounds
        if (ship_sprite.props.dead == false) {
            this.custom_sounds.player_explosion.play();
            this.custom_sounds.enemy_explosion.play();
        }
        // update the score
        player_ship.sprite.props.score += enemy.score;
        player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);
        // kill the player and the enemy. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives >= 1) {
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            enemy.play(enemy.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2 + 25;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                enemy.destroy();
            });
        }
        else {
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
        }
    });
    this.physics.add.collider(ai_ship.sprite, enemies_right.enemies_group, (ship_sprite, enemy) => {
        // play sounds
        if (ship_sprite.props.dead == false) {
            this.custom_sounds.player_explosion.play();
            this.custom_sounds.enemy_explosion.play();
        }
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
                ship_sprite.x = this.sys.canvas.width / 2 + 25;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                enemy.destroy()
            });
        }
        else {
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
        }
    });
}