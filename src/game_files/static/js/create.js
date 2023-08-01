// --- helper functions ---

/**
 * Create world objects and collider functions
 */
var icon;
 function create ()
 {	
     // console.log("create");
     // a log of all of the frames of the game
     frames = [];
     frame_number = 0;
     last_frame = -1;
     date = new Date();
     events = [];
     // scene = this.scene;
 
     // game.scale.fullScreenScaleMode = Phaser.ScaleManager.EXACT_FIT;
 
     // game.scale.fullScreenScaleMode = Phaser.ScaleManager.EXACT_FIT;
     // game.scale.scaleMode = Phaser.ScaleManager.EXACT_FIT;
     // game.scale.refresh();
 
     // game.input.onDown.add(gofull, this);
 
     // function gofull() {
     //     // if (game.scale.isFullScreen) {
     //     //     game.scale.stopFullScreen();
     //     // } else {
     //     //     game.scale.startFullScreen(false);
     //     // }
     //     game.scale.resize(window.innerWidth, window.innerHeight);
     // }
 
     // flag to tell when the game is over
     player_over = false;
     ai_over = false;
 
     // debug_text flag to run debugging text in developer tools
     debug_text = false;
 
     cursors = this.input.keyboard.createCursorKeys();
     space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
 
     keyA = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.A);
     keyS = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.S);
     keyD = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.D);
     keyW = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.W);
     keyG = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.G);
 
     // fullscreen
     keyF = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.F);
     keyF.on('down', function () {
         if (this.scale.isFullscreen) {
             this.scale.stopFullscreen();
         } else {
             this.scale.startFullscreen();
         }
     }, this);
 
     this.custom_sounds = {};   // add sound object so that we can easily access the sounds in the scene  
     this.custom_sounds.enemy_explosion = this.sound.add("audio_explosion", {volume: 0.05});
     this.custom_sounds.player_explosion = this.sound.add("audio_explosion", {volume: 0.1});
     this.custom_sounds.fire_ship = this.sound.add("audio_fire_ship", {volume: 0.05});
 
     //setting the left wall for the AI depending on what kind of agent it is
     // ai_ship = this.create_ship(0, "jordan", 1, this.sys.canvas.width / 4 + 400, 540, 5, "laser", 0, 30);
     // let ship1 = this.create_ship(1, "ship", 0, this.sys.canvas.width / 2, 540);
     // let ship2 = this.create_ship(2, "ship2", 0, this.sys.canvas.width / 4, 540);
     ai_ship = this.create_ship(0, "jordan", 1, this.sys.canvas.width / 2, 540, 5, "laser", 0, 30);
     let ship1 = this.create_ship(1, "ship", 0, this.sys.canvas.width / 4 - 100, 540);
     let ship2 = this.create_ship(2, "ship2", 0, this.sys.canvas.width / 2 + this.sys.canvas.width / 4 + 100, 540);
    //  let icon = this.load.image('icon', 'assets/images/first_place_icon.png');
     icon = this.physics.add.staticSprite(this.sys.canvas.width / 2 - 300, 560, 'icon');


     players.push(ship1);
     players.push(ship2);
     console.log(players);
     console.log("created ships");

 
 
     // var minX = 0;
     // if (mode == UNCOOPERATIVE) {
     //     minX = 425;
     //     ai_ship = this.create_ship("avery", 1, this.sys.canvas.width / 4 + 400, 540, 5, "laser", minX);
     // }
     // else  {
     //     ai_ship = this.create_ship("jordan", 1, this.sys.canvas.width / 4 + 400, 540, 5, "laser", minX);
     // }
 
     //creating the enemies on the left and right
     var enemy_rows = 5;
     //enemies_left = this.create_enemies(13, 30, 50, "a", 5, 10, "enemylaser", min_x = 0, max_x = 800);
     enemies_left = this.create_enemies(6, 30, 50, "a", 5, 10, "enemylaser", min_x = 0, max_x = 350); 
     enemies_right = this.create_enemies(6, 450, 50, "b", 5, 10, "enemylaser", min_x = 420, max_x = 600);

     // enemies_left = this.create_enemies(4, 30, 0, "a", 5, 10, "enemylaser", min_x = 0, max_x = 255);
     //enemies_middle = this.create_enemies(0, 100, 0, "c", 5, 10, "enemylaser", min_x = 280, max_x = 525);
 
     // if (ai_supporting == 0) {
     //     timer = 60;
     // }
 
     for (let s = 0; s < 2; s++) {
 
         // --> COLLIDERS <--
         // --> enemies hit by player_ship bullets 
         this.physics.add.overlap(enemies_left.enemies_group, players[s].bullets_group, (enemy, bullet) => {
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
                 if (s == 0) {
                     p1_score_text.setText("YOUR SCORE " + players[s].sprite.props.score);
                 } else if (s == 1) {
                     p2_score_text.setText("SHUTTER SCORE " + players[s].sprite.props.score);
                 }
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
         this.physics.add.collider(enemies_left.bullets_group, players[s].bullets_group, (enemy_bullet, ship_bullet) => {
             // hide both bullets 
             enemy_bullet.body.x = this.sys.canvas.width;
             enemy_bullet.body.y = this.sys.canvas.height;
             ship_bullet.body.x = this.sys.canvas.width;
             ship_bullet.body.y = this.sys.canvas.height;
             enemy_bullet.setActive(false);
             ship_bullet.setActive(false);
         });
 
         // --> enemies bullets hit player_ship
         this.physics.add.collider(players[s].sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
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
                     console.log("player dead");
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
                 if (s == 0) {
                     p1_score_text.setText("YOUR SCORE " + players[s].sprite.props.score);
                 } else if (s == 1) {
                     p2_score_text.setText("SHUTTER SCORE " + players[s].sprite.props.score);
                 }
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

     for (let s = 0; s < 2; s++) {
 
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
                if (s == 0) {
                    p1_score_text.setText("YOUR SCORE " + players[s].sprite.props.score);
                } else if (s == 1) {
                    p2_score_text.setText("SHUTTER SCORE " + players[s].sprite.props.score);
                }
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
                    console.log("player dead");
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
                if (s == 0) {
                    p1_score_text.setText("YOUR SCORE " + players[s].sprite.props.score);
                } else if (s == 1) {
                    p2_score_text.setText("SHUTTER SCORE " + players[s].sprite.props.score);
                }
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
             ai_ship.sprite.props.score+= enemy.score;



             // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);
 
             total_score += enemy.score;
             if (ai_supporting == 1) {
                 p1_score_text.setText("YOUR SCORE " + players[ai_supporting-1].sprite.props.score);
             } else if (ai_supporting == 2) {
                 p2_score_text.setText("SHUTTER SCORE " + players[ai_supporting-1].sprite.props.score);
             }
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
            ai_ship.sprite.props.score+= enemy.score;

            // player_ship.sprite.props.scoreText.setText("SCORE " + player_ship.sprite.props.score);

            total_score += enemy.score;
            if (ai_supporting == 1) {
                p1_score_text.setText("YOUR SCORE " + players[ai_supporting-1].sprite.props.score);
            } else if (ai_supporting == 2) {
                p2_score_text.setText("SHUTTER SCORE " + players[ai_supporting-1].sprite.props.score);
            }
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
             if (ai_supporting == 1) {
                 p1_score_text.setText("YOUR SCORE " + players[ai_supporting-1].sprite.props.score);
             } else if (ai_supporting == 2) {
                 p1_score_text.setText("SHUTTER SCORE " + players[ai_supporting-1].sprite.props.score);
             }
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
            events.push({frame: frame_number, killer: 'LEFT', killed: 'AI', type: 'COLLISION'});
            events.push({frame: frame_number, killer: 'AI', killed: 'LEFT', type: 'COLLISION'});
        }
    });
 
 
     var p1_score_text = this.add.bitmapText(this.sys.canvas.width/2, 3, 'PressStart2P_Green', '', 18);
     p1_score_text.originX = 0.5;
     p1_score_text.setText('YOUR SCORE 0');
 
     var p2_score_text = this.add.bitmapText(this.sys.canvas.width/2, 3, 'PressStart2P_Green', '', 18);
     p2_score_text.originX = 0.5;
     p2_score_text.originY = -1.5;
     p2_score_text.setText('SHUTTER SCORE 0');
 
 
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
         console.log(timer)
         timer = timer - 1;
         time = get_time(timer);
         timer_text.setText('TIMER ' + time)
         // if (timer == 0) {
         //     window.clearInterval(timer_interval);
         // }
     }, 1000)
     console.log('players score', players[0].sprite.props.score, players[1].sprite.props.score)

 
 
 
 }