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

    //ai_ship = this.create_ship(0, "jordan", 1, this.sys.canvas.width / 2, 540, 5, "laser", 0, 30);



   // ai_ship = this.create_ship(0, "jordan", 1, this.sys.canvas.width / 2, 540, 5, "laser", 0, 30);
    //enemies_left = this.create_enemies(6, 30, 50, "a", 5, 10, "enemylaser", min_x = 0, max_x = 350); 
    //enemies_right = this.create_enemies(6, 450, 50, "b", 5, 10, "enemylaser", min_x = 420, max_x = 600);

    //enemies_practice = this.create_enemies(5, 235, 0, "p", 5, 10, "enemylaser", min_x = 205, max_x = 595, [1, 1, 1]);


    // --> COLLIDERS <--
    // --> enemies hit by player_ship bullets 
    this.physics.add.overlap(enemies_right.enemies_group, players[s].bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.enemy_explosion.play();
        // enemy.play(enemy.explote_anim, true);
        // enemy.on('animationcomplete', () => {
        enemy.setVisible(false);
        
        // enemy.is_hit = true;
        // enemy.disableGameObject();

        // setTimeout(function () {
        //     console.log("this enemy " + 'enemy' + enemy.e + enemy.g + '_1');
        //     // console.log("Been 5 secs");
        //     // enemy = 500;
        //     // enemy.setTexture("enemy1a_1");
        //     enemy.is_hit = false;
        //     enemy.setTexture('enemy' + enemy.e + enemy.g + '_1');
        //     enemy.setVisible(true);
        //     // enemy.play('enemy' + enemy.e + enemy.g + '_move');
        //     // enemy.enableGameObject();
        // }, respawn_timer)

        // enemy.destroy();
            // enemy.visible = false;
            // enemy.visible = true;
            // await delay(3000);
            // enemy.visible = true;
            // setTimeout(function () {
            //     enemy.visible = true;
            // }, 5000);
        // });
        // hide the bullet 
        // bullet.body.x = this.sys.canvas.width;
        // bullet.body.y = this.sys.canvas.height;
        // bullet.setActive(false);
        // update the score     
        if (enemy.is_hit == false) {   
            bullet.body.x = this.sys.canvas.width;
            bullet.body.y = this.sys.canvas.height;
            bullet.setActive(false);
            players[s].sprite.props.score += enemy.score;
            //    player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score); 	

            total_score += enemy.score;
            // if (s == 0) {
            //     p1_score_text.setText("ORANGE SCORE " + players[s].sprite.props.score);
            // } else if (s == 1) {
            //     p2_score_text.setText("BLUE SCORE " + players[s].sprite.props.score);
            // }
            events.push({frame: frame_number, killer: 'PLAYER', killed: 'LEFT', type: 'SHOT'});
            enemy.is_hit = true;
            enemy.shot_frame = frame_number;
            num_enemies_shot = num_enemies_shot + 1;
        }
        // enemy.hit = true;
        // enemy.is_hit = true;
        enemy.time_hit = timer;
    });

    // --> enemies bullets hit ships bullets
    this.physics.add.collider(enemies_right.bullets_group, players[s].bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
        enemy_bullet.setActive(false);
        ship_bullet.setActive(false);
    });

    // --> enemies bullets hit player_ship
    this.physics.add.collider(players[s].sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);

        if (ship_sprite.props.invincible) { }
        // kill the player. The change in behavior takes place within the update function of the ship
        else if (ship_sprite.props.lives > 1) {
            // give the player 50 frames of invincibility
            ship_sprite.props.invincible = true;
            // ship_sprite.props.invincibility_timer = 50;
            ship_sprite.props.invincibility_timer = frame_number;
            events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'SHOT'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
            });
        }
        else {
            events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'SHOT'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            if (players[0].sprite.props.dead == true && players[1].sprite.props.dead == true) {
                //console.log("player dead");
                player_over = true;
            }   
            // player_over = true;
        }
    });

    // --> enemies hit player_ship
    this.physics.add.collider(players[s].sprite, enemies_left.enemies_group, (ship_sprite, enemy) => {
        if (!enemy.hit) {
            // play sounds
            this.custom_sounds.player_explosion.play();
            this.custom_sounds.enemy_explosion.play();
            // update the score
            // ship_sprite.props.score += enemy.score;
            // ship_sprite.props.scoreText.setText("SCORE " + ship_sprite.props.score);
            players[s].sprite.props.score += enemy.score
            total_score += enemy.score;
            // if (s == 0) {
            //     p1_score_text.setText("ORANGE SCORE " + players[s].sprite.props.score);
            // } else if (s == 1) {
            //     p2_score_text.setText("BLUE SCORE " + players[s].sprite.props.score);
            // }
            // p1_score_text.setText("SCORE " + total_score);
            // kill the player and the enemy. The change in behavior takes place within the update function of the ship
            if (ship_sprite.props.lives > 1) {
                ship_sprite.props.exploding = true;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            }
            else {
                ship_sprite.props.dead = true;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
                if (players[0].sprite.props.dead == true && players[1].sprite.props.dead == true) {
                    console.log("player dead")
                    player_over = true;
                }   
                // player_over = true;
            }
            ship_sprite.play(ship_sprite.explote_anim, true);
            enemy.play(enemy.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
            });
            enemy.on('animationcomplete', () => {
                enemy.destroy();
            });
            enemy.hit = true;
            events.push({frame: frame_number, killer: 'LEFT', killed: 'PLAYER', type: 'COLLISION'});
            events.push({frame: frame_number, killer: 'PLAYER', killed: 'LEFT', type: 'COLLISION'});
        }
    });
    
}

 // --> enemies hit by Ai ship's bullets
 this.physics.add.overlap(enemies_right.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
     // destroy the enemy
     this.custom_sounds.enemy_explosion.play();
     // enemy.play(enemy.explote_anim, true);
     // enemy.on('animationcomplete', () => {
     // enemy.destroy();
     enemy.setVisible(false);
     
     // enemy.is_hit = true;
     // enemy.disableGameObject();

     // setTimeout(function () {
     //     console.log("this enemy " + 'enemy' + enemy.e + enemy.g + '_1');
     //     // console.log("Been 5 secs");
     //     // enemy = 500;
     //     // enemy.setTexture("enemy1a_1");
     //     enemy.is_hit = false;
     //     enemy.setTexture('enemy' + enemy.e + enemy.g + '_1');
     //     enemy.setVisible(true);
     //     // enemy.play('enemy' + enemy.e + enemy.g + '_move');
     //     // enemy.enableGameObject();
     // }, respawn_timer)

     // });
     // enemy.visible = false;
     // hide the bullet 
     // bullet.body.x = this.sys.canvas.width;
     // bullet.body.y = this.sys.canvas.height;
     // bullet.setActive(false);
     // update the score
     if (enemy.is_hit == false) {
         bullet.body.x = this.sys.canvas.width;
         bullet.body.y = this.sys.canvas.height;
         bullet.setActive(false);
         players[ai_supporting-1].sprite.props.score += enemy.score;
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

     // setTimeout((function (scope) {
     //     return function() {
     //         // enemy = scope.physics.add.sprite(400, 300, 'enemy' + enemy.e + enemy.g + '_1').play('enemy' + enemy.e + enemy.g + '_move');
     //         // enemy.setOrigin(0.5, 1.0);
     //         // enemy.displayWidth = 50;
     //         // enemy.scaleY = enemy.scaleX;
     //         // enemy.body.maxVelocity.y = enemy.max_vel;
     //         // enemy.explote_anim = 'enemy' + enemy.e + enemy.g + '_exp';
     //         // // add extra parameters to know what is the position of the enemy in the grid
     //         // enemy.grid_row = Math.floor(i / enemy.num_horizontal);
     //         // for (var j=enemy.e-2; j >= 0; j--) {
     //         //     enemy.grid_row += num_rows[j];
     //         // }
     //         // enemy.grid_column = enemy.i % enemy.num_horizontal;
     //         // enemy.score = 10 //+ 10 * (num_rows[0] + num_rows[1] + num_rows[2] - 1 - enemy.grid_row);
     //         // enemy.hit = false;
     //         // enemy.visible = true;
     //         // enemies_left.enemies_group.add(enemy);
     //         console.log("wait 5 seconds");

     //     };
     // })(this), 5000);
 });

 this.physics.add.overlap(enemies_left.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
    // destroy the enemy
    this.custom_sounds.enemy_explosion.play();
    // enemy.play(enemy.explote_anim, true);
    // enemy.on('animationcomplete', () => {
    // enemy.destroy();
    enemy.setVisible(false);
    
    // enemy.is_hit = true;
    // enemy.disableGameObject();

    // setTimeout(function () {
    //     console.log("this enemy " + 'enemy' + enemy.e + enemy.g + '_1');
    //     // console.log("Been 5 secs");
    //     // enemy = 500;
    //     // enemy.setTexture("enemy1a_1");
    //     enemy.is_hit = false;
    //     enemy.setTexture('enemy' + enemy.e + enemy.g + '_1');
    //     enemy.setVisible(true);
    //     // enemy.play('enemy' + enemy.e + enemy.g + '_move');
    //     // enemy.enableGameObject();
    // }, respawn_timer)

    // });
    // enemy.visible = false;
    // hide the bullet 
    // bullet.body.x = this.sys.canvas.width;
    // bullet.body.y = this.sys.canvas.height;
    // bullet.setActive(false);
    // update the score
    if (enemy.is_hit == false) {
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        players[ai_supporting-1].sprite.props.score += enemy.score;
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

    // setTimeout((function (scope) {
    //     return function() {
    //         // enemy = scope.physics.add.sprite(400, 300, 'enemy' + enemy.e + enemy.g + '_1').play('enemy' + enemy.e + enemy.g + '_move');
    //         // enemy.setOrigin(0.5, 1.0);
    //         // enemy.displayWidth = 50;
    //         // enemy.scaleY = enemy.scaleX;
    //         // enemy.body.maxVelocity.y = enemy.max_vel;
    //         // enemy.explote_anim = 'enemy' + enemy.e + enemy.g + '_exp';
    //         // // add extra parameters to know what is the position of the enemy in the grid
    //         // enemy.grid_row = Math.floor(i / enemy.num_horizontal);
    //         // for (var j=enemy.e-2; j >= 0; j--) {
    //         //     enemy.grid_row += num_rows[j];
    //         // }
    //         // enemy.grid_column = enemy.i % enemy.num_horizontal;
    //         // enemy.score = 10 //+ 10 * (num_rows[0] + num_rows[1] + num_rows[2] - 1 - enemy.grid_row);
    //         // enemy.hit = false;
    //         // enemy.visible = true;
    //         // enemies_left.enemies_group.add(enemy);
    //         console.log("wait 5 seconds");

    //     };
    // })(this), 5000);
});


this.physics.add.overlap(enemies_right.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
    // destroy the enemy
    this.custom_sounds.enemy_explosion.play();
    // enemy.play(enemy.explote_anim, true);
    // enemy.on('animationcomplete', () => {
    // enemy.destroy();
    enemy.setVisible(false);
    
    // enemy.is_hit = true;
    // enemy.disableGameObject();

    // setTimeout(function () {
    //     console.log("this enemy " + 'enemy' + enemy.e + enemy.g + '_1');
    //     // console.log("Been 5 secs");
    //     // enemy = 500;
    //     // enemy.setTexture("enemy1a_1");
    //     enemy.is_hit = false;
    //     enemy.setTexture('enemy' + enemy.e + enemy.g + '_1');
    //     enemy.setVisible(true);
    //     // enemy.play('enemy' + enemy.e + enemy.g + '_move');
    //     // enemy.enableGameObject();
    // }, respawn_timer)

    // });
    // enemy.visible = false;
    // hide the bullet 
    // bullet.body.x = this.sys.canvas.width;
    // bullet.body.y = this.sys.canvas.height;
    // bullet.setActive(false);
    // update the score
    if (enemy.is_hit == false) {
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        players[ai_supporting-1].sprite.props.score += enemy.score;
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

    // setTimeout((function (scope) {
    //     return function() {
    //         // enemy = scope.physics.add.sprite(400, 300, 'enemy' + enemy.e + enemy.g + '_1').play('enemy' + enemy.e + enemy.g + '_move');
    //         // enemy.setOrigin(0.5, 1.0);
    //         // enemy.displayWidth = 50;
    //         // enemy.scaleY = enemy.scaleX;
    //         // enemy.body.maxVelocity.y = enemy.max_vel;
    //         // enemy.explote_anim = 'enemy' + enemy.e + enemy.g + '_exp';
    //         // // add extra parameters to know what is the position of the enemy in the grid
    //         // enemy.grid_row = Math.floor(i / enemy.num_horizontal);
    //         // for (var j=enemy.e-2; j >= 0; j--) {
    //         //     enemy.grid_row += num_rows[j];
    //         // }
    //         // enemy.grid_column = enemy.i % enemy.num_horizontal;
    //         // enemy.score = 10 //+ 10 * (num_rows[0] + num_rows[1] + num_rows[2] - 1 - enemy.grid_row);
    //         // enemy.hit = false;
    //         // enemy.visible = true;
    //         // enemies_left.enemies_group.add(enemy);
    //         console.log("wait 5 seconds");

    //     };
    // })(this), 5000);
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

      // --> enemies bullets hit AI ship's bullets
this.physics.add.collider(enemies_right.bullets_group, ai_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
        enemy_bullet.setActive(false);
        ship_bullet.setActive(false);
    });


 // enemies bullets hit AI ship's bullets
 this.physics.add.collider(ai_ship.sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
     // hide the bullet 
     bullet.body.x = this.sys.canvas.width;
     bullet.body.y = this.sys.canvas.height;
     bullet.setActive(false);
     if (ship_sprite.props.invincible) { }
     // kill the player. The change in behavior takes place within the update function of the ship
     else if (ship_sprite.props.lives > 1) {
         // give the player 50 frames of invincibility
         ship_sprite.props.invincible = true;
         ship_sprite.props.invincibility_timer = frame_number;
         events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
         this.custom_sounds.player_explosion.play();
         ship_sprite.props.exploding = true;
         ship_sprite.props.lives -= 1;
         ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
         ship_sprite.play(ship_sprite.explote_anim, true);
         ship_sprite.on('animationcomplete', () => {
             ship_sprite.x = this.sys.canvas.width / 2 + 25;
             ship_sprite.setTexture(ship_sprite.props.image_id);
             ship_sprite.props.exploding = false;
         });
     }
     else {
         events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
         this.custom_sounds.player_explosion.play();
         ship_sprite.props.exploding = true;
         ship_sprite.play(ship_sprite.explote_anim, true);
         ship_sprite.on('animationcomplete', () => {
             ship_sprite.setTexture(ship_sprite.props.image_id);
             ship_sprite.props.exploding = false;
             ship_sprite.props.lives -= 1;
             ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
             ship_sprite.props.dead = true;
         });
         ai_over = true;
         //console.log("AI OVER");
     }
 });


 this.physics.add.collider(ai_ship.sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
    // hide the bullet 
    bullet.body.x = this.sys.canvas.width;
    bullet.body.y = this.sys.canvas.height;
    bullet.setActive(false);
    if (ship_sprite.props.invincible) { }
    // kill the player. The change in behavior takes place within the update function of the ship
    else if (ship_sprite.props.lives > 1) {
        // give the player 50 frames of invincibility
        ship_sprite.props.invincible = true;
        ship_sprite.props.invincibility_timer = frame_number;
        events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
        this.custom_sounds.player_explosion.play();
        ship_sprite.props.exploding = true;
        ship_sprite.props.lives -= 1;
        ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
        ship_sprite.play(ship_sprite.explote_anim, true);
        ship_sprite.on('animationcomplete', () => {
            ship_sprite.x = this.sys.canvas.width / 2 + 25;
            ship_sprite.setTexture(ship_sprite.props.image_id);
            ship_sprite.props.exploding = false;
        });
    }
    else {
        events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
        this.custom_sounds.player_explosion.play();
        ship_sprite.props.exploding = true;
        ship_sprite.play(ship_sprite.explote_anim, true);
        ship_sprite.on('animationcomplete', () => {
            ship_sprite.setTexture(ship_sprite.props.image_id);
            ship_sprite.props.exploding = false;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.props.dead = true;
        });
        ai_over = true;
        //console.log("AI OVER");
    }
});

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


 // enemies bullets hit AI ship's bullets
 this.physics.add.collider(ai_ship.sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
     // hide the bullet 
     bullet.body.x = this.sys.canvas.width;
     bullet.body.y = this.sys.canvas.height;
     bullet.setActive(false);
     if (ship_sprite.props.invincible) { }
     // kill the player. The change in behavior takes place within the update function of the ship
     else if (ship_sprite.props.lives > 1) {
         // give the player 50 frames of invincibility
         ship_sprite.props.invincible = true;
         ship_sprite.props.invincibility_timer = frame_number;
         events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
         this.custom_sounds.player_explosion.play();
         ship_sprite.props.exploding = true;
         ship_sprite.props.lives -= 1;
         ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
         ship_sprite.play(ship_sprite.explote_anim, true);
         ship_sprite.on('animationcomplete', () => {
             ship_sprite.x = this.sys.canvas.width / 2 + 25;
             ship_sprite.setTexture(ship_sprite.props.image_id);
             ship_sprite.props.exploding = false;
         });
     }
     else {
         events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
         this.custom_sounds.player_explosion.play();
         ship_sprite.props.exploding = true;
         ship_sprite.play(ship_sprite.explote_anim, true);
         ship_sprite.on('animationcomplete', () => {
             ship_sprite.setTexture(ship_sprite.props.image_id);
             ship_sprite.props.exploding = false;
             ship_sprite.props.lives -= 1;
             ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
             ship_sprite.props.dead = true;
         });
         ai_over = true;
         //console.log("AI OVER");
     }
 });

      // enemies bullets hit AI ship's bullets
    this.physics.add.collider(ai_ship.sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        bullet.setActive(false);
        if (ship_sprite.props.invincible) { }
        // kill the player. The change in behavior takes place within the update function of the ship
        else if (ship_sprite.props.lives > 1) {
            // give the player 50 frames of invincibility
            ship_sprite.props.invincible = true;
            ship_sprite.props.invincibility_timer = frame_number;
            events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.x = this.sys.canvas.width / 2 + 25;
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
            });
        }
        else {
            events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'SHOT'});
            this.custom_sounds.player_explosion.play();
            ship_sprite.props.exploding = true;
            ship_sprite.play(ship_sprite.explote_anim, true);
            ship_sprite.on('animationcomplete', () => {
                ship_sprite.setTexture(ship_sprite.props.image_id);
                ship_sprite.props.exploding = false;
                ship_sprite.props.lives -= 1;
                ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
                ship_sprite.props.dead = true;
            });
            ai_over = true;
            //console.log("AI OVER");
        }
    });


 // --> enemies hit ai_ship
 this.physics.add.collider(ai_ship.sprite, enemies_left.enemies_group, (ship_sprite, enemy) => {
     if (!enemy.hit) {
         // play sounds
         this.custom_sounds.player_explosion.play();
         this.custom_sounds.enemy_explosion.play();
     
         // update the score
         // player_ship.sprite.props.score += enemy.score;
         // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);
         players[ai_supporting-1].sprite.props.score += enemy.score;
         total_score += enemy.score;
        //  if (ai_supporting == 1) {
        //      p1_score_text.setText("ORANGE SCORE " + players[ai_supporting-1].sprite.props.score);
        //  } else if (ai_supporting == 2) {
        //      p1_score_text.setText("BLUE SCORE " + players[ai_supporting-1].sprite.props.score);
        //  }
         // p1_score_text.setText("SCORE " + total_score);
         // kill the player and the enemy. The change in behavior takes place within the update function of the ship
         if (ship_sprite.props.lives > 1) {
             ship_sprite.props.exploding = true;
             ship_sprite.props.lives -= 1;
             ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
         }
         else {
             ship_sprite.props.dead = true;
             ship_sprite.props.lives -= 1;
             ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
             ai_over = true;
         }
         ship_sprite.play(ship_sprite.explote_anim, true);
         enemy.play(enemy.explote_anim, true);
         ship_sprite.on('animationcomplete', () => {
             ship_sprite.x = this.sys.canvas.width / 2 + 25;
             ship_sprite.setTexture(ship_sprite.props.image_id);
             ship_sprite.props.exploding = false;
         });
         enemy.on('animationcomplete', () => {
             enemy.destroy();
         });
         enemy.hit = true;
         events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'COLLISION'});
         events.push({frame: frame_number, killer: 'AI', killed: 'LEFT', type: 'COLLISION'});
     }
 });

 this.physics.add.collider(ai_ship.sprite, enemies_right.enemies_group, (ship_sprite, enemy) => {
    if (!enemy.hit) {
        // play sounds
        this.custom_sounds.player_explosion.play();
        this.custom_sounds.enemy_explosion.play();
    
        // update the score
        // player_ship.sprite.props.score += enemy.score;
        // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);
        players[ai_supporting-1].sprite.props.score += enemy.score;
        total_score += enemy.score;
        // if (ai_supporting == 1) {
        //     p1_score_text.setText("ORANGE SCORE " + players[ai_supporting-1].sprite.props.score);
        // } else if (ai_supporting == 2) {
        //     p1_score_text.setText("BLUE SCORE " + players[ai_supporting-1].sprite.props.score);
        // }
        // p1_score_text.setText("SCORE " + total_score);
        // kill the player and the enemy. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives > 1) {
            ship_sprite.props.exploding = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
        }
        else {
            ship_sprite.props.dead = true;
            ship_sprite.props.lives -= 1;
            ship_sprite.lives[ship_sprite.props.lives].setVisible(false);
            ai_over = true;
        }
        ship_sprite.play(ship_sprite.explote_anim, true);
        enemy.play(enemy.explote_anim, true);
        ship_sprite.on('animationcomplete', () => {
            ship_sprite.x = this.sys.canvas.width / 2 + 25;
            ship_sprite.setTexture(ship_sprite.props.image_id);
            ship_sprite.props.exploding = false;
        });
        enemy.on('animationcomplete', () => {
            enemy.destroy();
        });
        enemy.hit = true;
    }
});




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
     //console.log(timer)
     timer = timer - 1;
     time = get_time(timer);
     timer_text.setText('TIMER ' + time)
     // if (timer == 0) {
     //     window.clearInterval(timer_interval);
     // }
 }, 1000)





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