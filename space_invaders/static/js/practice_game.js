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
        // play sound
        this.custom_sounds.player_explosion.play();
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        // kill the player. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives >= 1) {
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 8;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
            });
        }
        else {
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
    for (var i=0; i < enemies_practice_sprites.length; i++) {
        if (enemies_practice_sprites[i].y > player_ship.sprite.y) {
            gameover = true;
        }
    }

    enemies_practice.update();

    // switch to first intermediate screen
    if (gameover || enemies_practice_sprites.length == 0) {
        this.scene.start('gameover_scene_practice');
    }
}


