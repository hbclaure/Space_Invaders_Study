// Space Invaders game built with Phaser 3
// This game is inspired by Lee Robinson's Space Invaders, https://leerob.io/blog/space-invaders-with-python
// And an adaptation by Simon Mendelsohn, https://zoo.cs.yale.edu/classes/cs490/19-20a/mendelsohn.simon.sjm225

// TO COMPLETE: 
// AI logic
// cooperative vs. noncooperative setting
// win/lose logic
// lives graphics

// --- helper functions ---

/**
 * Create bullets pool
 * @param {int} number number of bullets in the pool
 * @param {String} image_id image id for the bullets
 * @param {bool} increase_bbox increase the bullet bounding box by 3 times (makes it easier to detect collisions)
 */
function create_bullets_pool(number, image_id, increase_bbox=false) {
    bullets = this.physics.add.group({ // bullets pool. They become visible when a bullet is fired
        key: image_id,
        repeat: number,
        visible: false,
        active: false,
        collideWorldBounds: true,
    }); 
    // prevent the bullets from falling so that they don't kill one of the enemies by accident 
    // when they are created... 
    Phaser.Actions.Call(bullets.getChildren(), function(b) {
        b.body.setImmovable(true);
        b.body.setAllowGravity(false);
    });
    // increase bullet collision bounding box in case they are going down (makes the game a bit easier)
    if (increase_bbox) {
        Phaser.Actions.Call(bullets.getChildren(), function(b) {
            b.body.setSize(b.body.width*3 , b.body.height*0.5, true);
        });
    }
    return bullets
}

/**
 * Helper function to fire a bullet
 * @param {Phaser.GameObjects.GameObject} bullets_group bullets group (as output by create_bullets_pool())
 * @param {int} x horizontal coordinate for the bullet
 * @param {int} y vertical coordinate for the bullet
 * @param {int} direction -1 or 1 depending on whether we want the bullet to move up or down
 * @param {int} speed magnitude of the bullet velocity
 */
function fire_bullet(bullets_group, x, y, direction, speed = 500) {
    // Scans the group for the first member that has an Phaser.GameObjects.GameObject#active state set to false,
    // and assign position x and y
    var bullet = bullets_group.get(x, y);
    if (bullet) {
        // set bullet position and speed
        bullet.body.originX = 0.5;
        bullet.body.originY = 1.0;
        bullet.body.velocity.y = speed * direction;
        bullet.body.allowGravity = true;
        // make bullet visible
        bullet.setActive(true);
        bullet.setVisible(true);
        bullet.body.setImmovable(false);
        bullet.body.setAllowGravity(true);
        // setup callback to hide bullet when it reaches the end of the screen
        bullet.body.onWorldBounds = true;                    
        bullet.body.world.on('worldbounds', function(body) {
            if (body.gameObject == bullet) {
                bullet.setVisible(false);
                bullet.setActive(false);
                bullet.body.setImmovable(true);
                bullet.body.setAllowGravity(false);
            }
        })
    }
}

/**
 * Helper function to fire a bullet from a group of enemies
 * @param {Phaser.GameObjects.GameObject} enemies group of enemies
 * @param {Phaser.GameObjects.GameObject} bullets group of bullets that the enemies can fire
 */
function fire_enemy_bullet(enemies, bullets) {
    // find valid columns (those that still have enemies in)
    var num_valid = 0;
    var valid_columns = [];
    for (var i=0; i<enemies.num_columns; i++) {
        valid_columns.push(0);
    }
    var children = enemies.getChildren();
    for (var i = 0; i < children.length; i++) {
        col = children[i].grid_column;
        if ([valid_columns[col]] == 0) {
            valid_columns[col] = 1;
            num_valid += 1;
        }
        if (num_valid == enemies.num_columns) break;
    }
    // turn valid_columns to indexes
    var valid_index = [];
    for (var i=0; i<valid_columns.length; i++) {
        if (valid_columns[i] == 1)
            valid_index.push(i);
    }
    // sample a colum
    var chosen_index = Math.floor(Math.random() * num_valid);
    var col = valid_index[chosen_index];
    // find lowest row with an enemy for the chosen column
    var enemy = null;
    for (var i = 0; i < children.length; i++) {
        if (children[i].grid_column == col && (enemy == null || enemy.grid_row < children[i].grid_row)) {
            enemy = children[i];
        }
    }
    // fire bullet for that enemy
    if (enemy != null) {
        fire_bullet(bullets, enemy.body.x + enemy.body.width * 0.5, enemy.body.y + 50, 1, 50);
    }
}

/**
 * Create ship object
 * @param {string} image_id Image ID for the ship
 * @param {int} x horizontal coordinate for the ship (moves horizontally)
 * @param {int} y vertical coordinate for the ship (fixed)
 * @param {int} speed speed for the ship
 * @param {int} min_x minimum allowed horizontal position for the ship
 * @param {int} max_x maximum allowed horizontal position for the ship
 * @param {int} shooting_direction shooting direction (-1 means up, 1 means down)
 * @param {String} bullet_image_id image id for the bullet
 * @returns object with ship image, corresponding bullets group, and update function for handling actions
 */
function create_ship(image_id="ship", type = 0, x = 200, y = 540, speed = 5, bullet_image_id = "laser", min_x = 0) {
            
    var canvas_width = this.sys.canvas.width;
    var canvas_height = this.sys.canvas.height;
    var sprite = this.physics.add.sprite(x, y, image_id).setOrigin(0.5, 1.0); 
    sprite.body.setImmovable(true);
    sprite.body.setAllowGravity(false);
    sprite.body.setSize(sprite.width*0.8 , sprite.height, true);
    sprite.props = {} // add properties object to ship sprite
    sprite.props.speed = speed;
    sprite.props.dead = false;
    sprite.props.lives = 3;
    var obj_width = sprite.displayWidth;     

    var bullets = this.create_bullets_pool(30, bullet_image_id);
    var sound = this.custom_sounds.fire_ship;

    return {
        sprite: sprite,                                         //!< image object for the ship
        bullets_group: bullets,                                 //!< bullets shot by the shi
        min_x: min_x,
        update(move_left, move_right, shoot)                    //!< update the ship state based on requested actions   
        {          
            // do nothing if the ship has been killed!
            if (type == 0) {
                if (this.sprite.props.dead) {
                    this.sprite.alpha = 0.35;
                    return;
                }      
                // update position
                if (move_left) {
                    this.sprite.x = Math.max(this.sprite.x - this.sprite.props.speed, obj_width + min_x);
                } else if (move_right) {
                    this.sprite.x = Math.min(this.sprite.x + this.sprite.props.speed, canvas_width - obj_width);
                } 
                // add bullet
                if (shoot) {
                    fire_bullet(this.bullets_group, this.sprite.x, this.sprite.y - 50, -1);
                    sound.play();
                }
            }
            else {
                if (this.sprite.props.dead) {
                    this.sprite.alpha = 0.35;
                    return;
                } 
                if (shoot) {
                    fire_bullet(this.bullets_group, this.sprite.x, this.sprite.y - 50, -1);
                    sound.play();
                }

                if (move_left) {
                    this.sprite.x = Math.max(this.sprite.x - this.sprite.props.speed, obj_width + min_x);
                } else if (move_right) {
                    this.sprite.x = Math.min(this.sprite.x + this.sprite.props.speed, canvas_width - obj_width);
                } 

            } 
        },
    };
}



/**
 * Create a group of enemies
 * @param {int} num_horizontal number of columns in the group 
 * @param {int} x horizontal position of the group
 * @param {int} y vertical position of the group
 * @param {int} vertical_speed vertical speed (should be positive)
 * @param {int} horizontal_speed horizontal speed (should be positive)
 * @param {String} bullet_image_id image id for the enemy bullets
 * @param {int} min_x minimum horizontal position for an enemy (when it reaches this edge, it switches direction)
 * @param {int} max_x maximum horizontal position for an enemy (when it reaches this edge, it switches direction)
 * @param {Array} num_rows number of rows per enemy (1, 2, and 3). Must be an array with 3 ints.
 */
function create_enemies(num_horizontal = 5, x, y, g = "a", max_vel = 5, horizontal_speed = 10, 
                        bullet_image_id = "enemylaser", min_x = 0, max_x = 400, num_rows = [1, 2, 2])
{
    var canvas_width = this.sys.canvas.width                                    //!< width of the canvas
    var explosions = ['explosionpurple', 'explosionblue', 'explosiongreen'];    //!< name of explosion for each enemy (from 1 to 3)
    var enemies = this.physics.add.group();                                     // group of enemies

    // create animations for all enemies, animations for explosions, and groups of robots
    for (var e=1; e<4; e++) {
        // animation for the enemy
        this.anims.create({
            key: 'enemy' + e + g + '_move',
            frames: [
                { key: 'enemy' + e + g + '_1' },
                { key: 'enemy' + e + g + '_2' },
            ],
            frameRate: 2,
            repeat: -1
        });
        // animation for the enemy explosion
        this.anims.create({
            key: 'enemy' + e + g + '_exp',
            frames: [{ key: explosions[e - 1] }],
            frameRate: 10,
        });
        // add actual enemies to the enemies group 
        for (var i=0; i<num_horizontal * num_rows[e - 1]; i++) {
            enemy = this.physics.add.sprite(400, 300, 'enemy' + e + g + '_1').play('enemy' + e + g + '_move');
            enemy.setOrigin(0.5, 1.0);
            enemy.displayWidth = 50;
            enemy.scaleY = enemy.scaleX;
            enemy.body.maxVelocity.y = max_vel;
            enemy.explote_anim = 'enemy' + e + g + '_exp';
            // add extra parameters to know what is the position of the enemy in the grid
            enemy.grid_row = Math.floor(i / num_horizontal);
            for (var j=e-2; j >= 0; j--) {
                enemy.grid_row += num_rows[j];
            }
            enemy.grid_column = i % num_horizontal;
            enemies.add(enemy);
        }
    }
    // align group enemies in a grid
    Phaser.Actions.GridAlign(enemies.getChildren(), 
    { width: num_horizontal, height: 5, cellWidth: 60, cellHeight: 50, position: Phaser.Display.Align.CENTER, x: x, y: y });
    // set initial velocity for group
    Phaser.Actions.Call(enemies.getChildren(), function(e) {
        e.setVelocityX(-horizontal_speed)
    })
    // store number of columns in the grid
    var children = enemies.getChildren();
    enemies.num_columns = num_horizontal;
    enemies.num_rows = children[children.length - 1].grid_row;

    // create bullets pool
    var bullets = this.create_bullets_pool(30, bullet_image_id, true);
    var sound = this.custom_sounds.fire_enemy;

    // create timer to fire enemy bullets
    enemies.fire_timer = this.time.addEvent({ delay: Phaser.Math.Between(700, 1000), loop: true, 
                                              callback: () => { 
                                                    fire_enemy_bullet(enemies, bullets);
                                              } });

    return {
        enemies_group: enemies,                //!< enemies group
        bullets_group: bullets,                //!< bullets group
        update(margin = 10)                    //!< update the enemies state
        {
            // move right?
            left_enemy = this.enemies_group.getChildren().find(function(e){
                return e.body.x < min_x;
            });
            if (typeof left_enemy != 'undefined') {
                Phaser.Actions.Call(this.enemies_group.getChildren(), function(e) {
                    e.setVelocityX(horizontal_speed)
                })
            } else { // move left?
                right_enemy = this.enemies_group.getChildren().find(function(e){
                    return e.body.x + e.body.width > max_x;
                });
                if (typeof right_enemy != 'undefined') {
                    Phaser.Actions.Call(this.enemies_group.getChildren(), function(e) {
                        e.setVelocityX(-horizontal_speed)
                    })
                }
            }
        },
    };
}


// --- actual game ---

/**
 * Phaser 3 game configuration 
 */
var config = {
    type: Phaser.AUTO,
    width: 800,
    height: 600,
    physics: {
        default: 'arcade',
        fps: 30,
        arcade: {
            debug: false, // set to true to enable physics visualization
            gravity: { y: 100 }
        }
    },
    scene: {
        preload: preload,
        create: create,
        update: update,
        extend: {
            fire_bullet: fire_bullet,
            create_bullets_pool: create_bullets_pool,
            create_ship: create_ship,
            create_enemies: create_enemies,
        }
    },
};

var game = new Phaser.Game(config);     //!< game object
var mode;
var cursors;                            //!< keyboard access
var space_key;                          //!< space key
var player_ship;                              //!< player_ship
var ai_ship;
var enemies_left;                           //!< enemies
var enemies_right;
var shift_key;
var all_enemies;
var shoot;
var move_left;
var move_right;
var hit;
var left_final;
var right_final;
var gameover;

/**
 * Preload assets for the game
 */
function preload ()
{

    // load imagess
    // this.load.setBaseURL('..');
    this.load.image('ship', 'assets/images/ship.png');
    this.load.image('avery', 'assets/images/avery.png');
    this.load.image('jordan', 'assets/images/jordan.png');
    this.load.image('mystery', 'assets/images/mystery.png');

    this.load.image('enemy1a_1', 'assets/images/enemy1_1.png');
    this.load.image('enemy1a_2', 'assets/images/enemy1_2.png');
    this.load.image('enemy2a_1', 'assets/images/enemy2_1.png');
    this.load.image('enemy2a_2', 'assets/images/enemy2_2.png');
    this.load.image('enemy3a_1', 'assets/images/enemy3_1.png');
    this.load.image('enemy3a_2', 'assets/images/enemy3_2.png');

    this.load.image('enemy1b_1', 'assets/images/enemy1_1.png');
    this.load.image('enemy1b_2', 'assets/images/enemy1_2.png');
    this.load.image('enemy2b_1', 'assets/images/enemy2_1.png');
    this.load.image('enemy2b_2', 'assets/images/enemy2_2.png');
    this.load.image('enemy3b_1', 'assets/images/enemy3_1.png');
    this.load.image('enemy3b_2', 'assets/images/enemy3_2.png');

    this.load.image('explosionblue', 'assets/images/explosionblue.png');
    this.load.image('explosiongreen', 'assets/images/explosiongreen.png');
    this.load.image('explosionpurple', 'assets/images/explosionpurple.png');
    this.load.image('laser', 'assets/images/laser.png');
    this.load.image('enemylaser', 'assets/images/enemylaser.png');

    // load audio
    this.load.audio("audio_explosion", "assets/sounds/shipexplosion.wav");
    this.load.audio("audio_fire_ship", "assets/sounds/shoot.wav");
}

/**
 * Create world objects and collider functions
 */
function create ()
{
    gameover = false;

    //this.physics.world.setFPS(1);
    //mode: 0 - cooperative, 1 - noncooperative
    mode = 1;

    cursors = this.input.keyboard.createCursorKeys();
    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
    shift_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SHIFT);

    this.custom_sounds = {};   // add sound object so that we can easily access the sounds in the scene  
    this.custom_sounds.explosion = this.sound.add("audio_explosion", {volume: 0.1});
    this.custom_sounds.fire_ship = this.sound.add("audio_fire_ship", {volume: 0.1});

    player_ship = this.create_ship("ship", 0, this.sys.canvas.width / 4, 540);

    var minX = 0;
    if (mode == 1) {
        minX = 400;
    }

    ai_ship = this.create_ship("avery", 1, this.sys.canvas.width / 4 + 400, 540, 5, "laser", minX);

    enemies_left = this.create_enemies(5, 30, 0, "a");
    enemies_right = this.create_enemies(5, 430, 0, "b", 5, 10, "enemylaser", min_x = 410, max_x = 800);

    // --> logic to create a ship group (BROKEN)
    // ship_group = this.physics.add.group();
    // ship_group.add(player_ship.sprite);
    // ship_group.add(ai_ship.sprite);

    // ai_ship.sprite.body.setAllowGravity(false);
    // player_ship.sprite.body.setAllowGravity(false);

     // this.physics.add.collider(enemies_left.bullets_group, ship_group, (bullet, ship) => {
    //     bullet.body.x = this.sys.canvas.width;
    //     bullet.body.y = this.sys.canvas.height;

    //     ship.props.dead = true;
    // });

    // --> COLLIDERS <--
    // --> enemies hit by player_ship bullets 
    this.physics.add.collider(enemies_left.enemies_group, player_ship.bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.explosion.play();
        enemy.play(enemy.explote_anim, true);
        enemy.on('animationcomplete', () => {
            enemy.destroy();
        });
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
    });
	this.physics.add.collider(enemies_right.enemies_group, player_ship.bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.explosion.play();
        enemy.play(enemy.explote_anim, true);
        enemy.on('animationcomplete', () => {
            enemy.destroy();
        });
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
    });

    // --> enemies hit by Ai ship's bullets
    this.physics.add.collider(enemies_left.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.explosion.play();
        enemy.play(enemy.explote_anim, true);
        enemy.on('animationcomplete', () => {
            enemy.destroy();
        });
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
    });
	this.physics.add.collider(enemies_right.enemies_group, ai_ship.bullets_group, (enemy, bullet) => {
        // destroy the enemy
        this.custom_sounds.explosion.play();
        enemy.play(enemy.explote_anim, true);
        enemy.on('animationcomplete', () => {
            enemy.destroy();
        });
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
    });

    // --> enemies bullets hit ships bullets
    this.physics.add.collider(enemies_left.bullets_group, player_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
    });
    this.physics.add.collider(enemies_right.bullets_group, player_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
    });

    // --> enemies bullets hit AI ship's bullets
    this.physics.add.collider(enemies_left.bullets_group, ai_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
    });
    this.physics.add.collider(enemies_right.bullets_group, ai_ship.bullets_group, (enemy_bullet, ship_bullet) => {
        // hide both bullets 
        enemy_bullet.body.x = this.sys.canvas.width;
        enemy_bullet.body.y = this.sys.canvas.height;
        ship_bullet.body.x = this.sys.canvas.width;
        ship_bullet.body.y = this.sys.canvas.height;
    });

    // --> enemies bullets hit player_ship
    this.physics.add.collider(player_ship.sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        // kill the enemy. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives >= 1) {
            ship_sprite.props.lives -= 1;
            ship_sprite.x = this.sys.canvas.width / 4;
        }
        else {
            ship_sprite.props.dead = true;
            gameover = true;
        }
    });

    this.physics.add.collider(player_ship.sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
        // hide the bullet 
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
        // kill the enemy. The change in behavior takes place within the update function of the ship
        if (ship_sprite.props.lives >= 1) {
            ship_sprite.props.lives -= 1;
            ship_sprite.x = this.sys.canvas.width / 4;
        }
        else {
            ship_sprite.props.dead = true;
            gameover = true;
        }
    });

    // --> enemies bullets hit ai_ship
    // this.physics.add.collider(ai_ship.sprite, enemies_right.bullets_group, (ship_sprite, bullet) => {
    //     // hide the bullet 
    //     bullet.body.x = this.sys.canvas.width;
    //     bullet.body.y = this.sys.canvas.height;
    //     // kill the enemy. The change in behavior takes place within the update function of the ship
    //     // ship_sprite.props.dead = true;
    //     if (ship_sprite.props.lives >= 1) {
    //         ship_sprite.props.lives -= 1;
    //         ship_sprite.x = this.sys.canvas.width / 4 + 400;
    //     }
    //     else {
    //         ship_sprite.props.dead = true;
    //     }
    // });

    // this.physics.add.collider(ai_ship.sprite, enemies_left.bullets_group, (ship_sprite, bullet) => {
    //     // hide the bullet 
    //     bullet.body.x = this.sys.canvas.width;
    //     bullet.body.y = this.sys.canvas.height;
    //     // kill the enemy. The change in behavior takes place within the update function of the ship
    //     // ship_sprite.props.dead = true;
    //     if (ship_sprite.props.lives >= 1) {
    //         ship_sprite.props.lives -= 1;
    //         ship_sprite.x = this.sys.canvas.width / 4 + 400;
    //     }
    //     else {
    //         ship_sprite.props.dead = true;
    //     }
    // });

}

/**
 * Update the state of the game
 */
function update ()
{
    // update the ship
    player_ship.update(cursors.left.isDown, cursors.right.isDown, this.input.keyboard.checkDown(space_key, 500));
    // manual control of ai update
    //ai_ship.update(cursors.up.isDown, cursors.down.isDown, this.input.keyboard.checkDown(shift_key, 500));

    // ai ship shoots randomly, 1/1000 
    shoot = false;
    var random = Math.floor(Math.random() * 250 + 1);
    //console.log("random: " + random);
    if (random == 1) {
        shoot = true;
    }

    // ---------- start AI logic

    left_final = false;
    right_final = false;

    var left_enemy = this.sys.canvas_width;
    var right_enemy = 0;

    move_left = move_right = true;
    hit = false;

    // --> checking where the enemies are
    var enemies_left_sprites = enemies_left.enemies_group.getChildren();
    for (var i=0; i < enemies_left_sprites; i++) {
        if (enemies_left_sprites[i].body.x > right_enemy) {
            right_enemy = enemies_left_sprites[i].body.x
        }
        if (enemies_left_sprites[i].body.x < left_enemy) {
            left_enemy = enemies_left_sprites[i].body.x
        }
    }

    var enemies_right_sprites = enemies_right.enemies_group.getChildren();
    for (var i=0; i < enemies_right_sprites; i++) {
        if (enemies_right_sprites[i].body.x > right_enemy) {
            right_enemy = enemies_left_sprites[i].body.x
        }
        if (enemies_right_sprites[i].body.x < left_enemy) {
            left_enemy = enemies_left_sprites[i].body.x
        }
    }

    // --> checking where the bullets are/if a bullet is about to hit the ai ship
    var bullets_right = enemies_right.bullets_group.getChildren();
    // console.log(bullets_left.length);
    for(var i=0; i < bullets_right.length; i++){
        // console.log(bullets_left[i].body.x);
        if (bullets_right[i].body.y < 300 || bullets_right[i].visible == false) {
            continue;
        }
        var x_diff = bullets_right[i].body.x - ai_ship.sprite.x;
        //console.log(ai_ship.sprite.x);
        if (x_diff > -30 && x_diff < -1) {
            move_left = false;
        }
        if (x_diff >= -1 && x_diff <= 50) {
            hit = true;
        }
        if (x_diff > 50 && x_diff < 80) {
            move_right = false;
        }
    }

    // var bullets_left = enemies_left.bullets_group.getChildren();
    // for(var i=0; i < bullets_left.length; i++){
    //     // console.log(bullets_left[i].body.x);
    //     if (bullets_left[i].visible == false) {
    //         continue;
    //     }
    //     var diff = bullets_left[i].body.x - ai_ship.sprite.x;
    //     //console.log(ai_ship.sprite.x);
    //     if (diff > -30 && diff < -1) {
    //         move_left = false;
    //     }
    //     if (diff >= -1 && diff <= 50) {
    //         hit = true;
    //     }
    //     if (diff > 50 && diff < 80) {
    //         move_right = false;
    //     }
    // }

    //console.log("left: " + move_left + ". right: " + move_right + ", hit: " + hit);

    // --> deciding which direction to move
    if (move_left && move_right && hit && ai_ship.sprite.x < ai_ship.min_x + 10) {
        right_final = true;
        console.log("1");
    }
    else if (move_left && move_right && hit && ai_ship.sprite.x > this.sys.canvas_width - 60) {
        left_final = true;
        console.log("2");
    }
    else if (move_left && hit && !move_right) {
        left_final = true;
        console.log("3");
        //console.log("left");
    }
    else if (move_right && hit && !move_left) {
        right_final = true;
        console.log("4");
        //console.log("right");
    }
    else if (ai_ship.sprite.x > right_enemy && move_left && ai_ship.sprite.x > ai_ship.min_x && !move_right) {
        left_final = true;
        console.log("5");
    }
    else if ((ai_ship.sprite.x < left_enemy + 10 && move_right) || (ai_ship.sprite.x < this.sys.canvas_width && move_right) && !move_left) {
        right_final = true;
        console.log("6");
    }

    console.log("left: " + left_final + ", right: " + right_final);

    ai_ship.update(left_final, right_final, shoot);

    // ---------- end AI logic

    // update the enemies
    enemies_left.update();
    enemies_right.update();



}