/**
All the helper functions and global variables for space invaders
**/

// set image width; height will be matched accordingly

SHIP_BULLET_SPEED = 350

var width = 500;
var height = 0;

var recording = null;

var streaming = false;

var video = null;
var canvas = null;
var photo = null;
var image = null;
var startTimeM = new Date().getTime();
var nowTime;
var millis;
var gameBlob;
var imgBlob;

function startup() {
    video = document.getElementById('video');
    canvas = document.getElementById('canvas');
    photo = document.getElementById('photo');

    if(display_vid=='on'){
        video.style.display='block'
    }

    navigator.mediaDevices.getUserMedia({ video: true, audio: false })
    .then(function(stream) {
        // copy.srcObject = stream;
        //copy.play();
        video.srcObject = stream;
        video.play();
    })
    .catch(function(err) {
        console.log("An error occurred: " + err);
        webcam_off_error();
    });
    
    video.addEventListener('canplay', function(ev){
        if (!streaming) {

            // maintain aspect ratio
            height = video.videoHeight / (video.videoWidth/width);
            ratio = video.videoWidth/width

            ratio_data = {
                ratio: ratio,
            };
            console.log('image logged')

            video.setAttribute('width', width/2);
            video.setAttribute('height', height/2);
            canvas.setAttribute('width', width);
            canvas.setAttribute('height', height);
            streaming = true;
        }
    }, false);
    //save_image_loop()
}

function webcam_off_error(){
    canvas.style.display='block';
    var ctx2 = canvas.getContext('2d');
    ctx2.beginPath();
    ctx2.fillStyle="red";
    ctx2.font = "15pt sans-serif";
    ctx2.fillText("ERROR: PLEASE ENABLE", 20, 50);
    ctx2.fillText("YOUR WEBCAM", 20, 75);
}

var startTime;
function save_image_loop(stage=1) {
    startTime = new Date().getTime();
    if (recording) {
      clearInterval(recording);
    }
    recording = setInterval(function(){
        logpicture(stage, frame_number);
        if (stage == 1) {
            loggame(frame_number);
        }
        if ((stage==2 || stage==0) && new Date().getTime() - startTime >= 10000) {
            clearInterval(recording);
            console.log('stopped recording');
        }
    }, 66);
}

// log user video frame
function logpicture(stage=1,current_frame_number) {
    // stage 0: start, 1: in-game, 2: end
    var context = canvas.getContext('2d');
    if (width && height) {
        canvas.width = width;
        canvas.height = height;
        context.drawImage(video, 0, 0, width, height);
        nowTime = new Date().getTime();
        millis = nowTime - startTimeM;
        //sockets.image.send(JSON.stringify({'img':canvas.toDataURL('image/jpeg'),'frame_number':frame_number,'stage':stage,'millis':millis}))
        canvas.toBlob(function(blob) {
            imgBlob = new Blob([current_frame_number,'z',stage,'y',millis,blob]);
            sockets.image.send(imgBlob);
        },'image/jpeg');
    }
}

// record game frames
function loggame(current_frame_number) {
    //sockets.game.send(JSON.stringify({'img':game.canvas.toDataURL('image/jpeg',0.1),'frame_number':frame_number}))
    nowTime = new Date().getTime();
    millis = nowTime - startTimeM;
    game.canvas.toBlob(function(blob) {
        gameBlob = new Blob([current_frame_number,'z',millis,blob]);
        sockets.game.send(gameBlob);
    }, 'image/jpeg',0.1);
}

window.addEventListener('load', startup, false);


// The different game modes
const PRACTICE = 0;
const COOPERATIVE_EARLY = 1;
const COOPERATIVE_LATE = 2;
const UNCOOPERATIVE = 3;

const static_url = '';

var mode;                               //!< game mode
var cursors;                            //!< keyboard access
var space_key;                          //!< space key
var enter_key;                          //!< enter key
var player_ship;                        //!< player_ship
var ai_ship;                            //!< ai_ship
var enemies_left;                       //!< enemies
var enemies_right;
var enemies_practice;
var debug_text;
var game_log = [];                      //!< a log of all the information from this game
var events;
var frames;                             //!< the frames of this game
var frame_number = 0;                       //!< the number of the current frame
var last_frame;                         //
var previous_shots_time = [];                //!< the times of the last 5 shots the player fired
var rounds_played = 0;                  //!< number of rounds that they have played
var player_score;                       //!< total player score (accumulated over multiple rounds)
var ai_score;                           //!< total ai score (accumulated over multiple rounds)

var date;                               //!< date
var player_id;                          //!< unique ID
var game_num;                           //!< game number
var display_vid;

var max_player_frequency = 600;
var max_ai_frequency = max_player_frequency * 0.6;


/**
Function to find the game id and game mode (which are passed as GET parameters)
Modified from code found at: https://stackoverflow.com/questions/5448545/how-to-retrieve-get-parameters-from-javascript
**/
function findGetParameter(parameterName) {
    var result = null,
        tmp = [];
    var items = location.search.substr(1).split("&");
    for (var index = 0; index < items.length; index++) {
        tmp = items[index].split("=");
        if (tmp[0] === parameterName) result = tmp[1];
    }
    return result;
}

player_id = findGetParameter('id') ? findGetParameter('id') : 'UNDEFINED';
mode = findGetParameter('mode'); 
mode = (mode && !isNaN(mode) && parseInt(mode, 10) >= 0 && parseInt(mode, 10) <= 3) ? parseInt(mode, 10) : COOPERATIVE_EARLY;
game_num = findGetParameter('game') ? findGetParameter('game') : 0;
display_vid = findGetParameter('v') ? findGetParameter('v') :'off';

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
        // make bullet visible
        bullet.setActive(true);
        bullet.setVisible(true);
        bullet.body.setImmovable(false);
        bullet.body.setAllowGravity(false);
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
    enemies.num_valid_columns = num_valid;
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
        fire_bullet(bullets, enemy.body.x + enemy.body.width * 0.5, enemy.body.y + 50, 1, 100);
    }
}

/**
 * Create ship object
 * @param {string} image_id Image ID for the ship
 * @param {int} type type of ship (0 for human, 1 for AI)
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
    sprite.props.score = 0;
    var font_type = (image_id == 'ship') ? 'PressStart2P_Purple' : (image_id == 'avery') ? 'PressStart2P_Orange' : 'PressStart2P_Gray';
    // sprite.props.scoreText = this.add.bitmapText(x, 3, font_type, 'SCORE 0', 20);
    // sprite.props.lives = 3;
    sprite.props.scoreText = this.add.bitmapText(x+22, 3, font_type, 'SCORE 0', 18);
    sprite.props.lives = 4;
    sprite.props.image_id = image_id;
    sprite.props.invincible = false;
    sprite.props.invincibility_timer = 0;
    sprite.props.last_shot_frame = 0;
    sprite.props.last_shot_time = 0;
    sprite.props.shot_cooldown = 35;
    sprite.props.exploding = false;
    sprite.explote_anim = image_id + '_exp';

    // animation for the player/ai explosions
    var explosion = (image_id == 'ship') ? 'shipexplosion' : (image_id == 'avery') ? 'averyexplosion' : 'jordanexplosion';


    if (!(this.anims.get(image_id + '_exp'))) { 
        this.anims.create({
            key: image_id + '_exp',
            frames: [{ key: explosion } ],
            frameRate: 10,
        });
    }
    

    
    sprite.lives = [] // add sprites to display lives
    var life_x = x - 200;
    this.add.bitmapText(life_x, 3, font_type, 'LIVES', 20);
    for (i = 0; i < sprite.props.lives; i++) {
        var life = this.physics.add.sprite(life_x + 125 + 25 * i, 25, image_id).setOrigin(0.5, 1.0);
        life.body.setImmovable(true);
        life.body.setAllowGravity(false);
        life.body.setSize(life.width * 0.4 , life.height * 0.5, true);
        life.setScale(0.5);
        sprite.lives.push(life);
    }

    var obj_width = sprite.displayWidth;     

    var bullets = this.create_bullets_pool(3, bullet_image_id);
    var sound = this.custom_sounds.fire_ship;

    return {
        sprite: sprite,                                         //!< image object for the ship
        bullets_group: bullets,                                 //!< bullets shot by the ship
        min_x: min_x,
        update(move_left, move_right, shoot)                    //!< update the ship state based on requested actions   
        {          
            // do nothing if the ship has been killed!
            if (this.sprite.props.dead) {
                this.sprite.setVisible(false);
                this.sprite.x = 0;
                this.sprite.y = 0;
                return;
            }
            // invincible sprite: don't let move, decrement timer, make clear
            if (this.sprite.props.invincible) {
                this.sprite.alpha = 0.35
                this.sprite.props.invincibility_timer -= 1;
                if (this.sprite.props.invincibility_timer == 0) {
                    this.sprite.props.invincible = false;
                    this.sprite.alpha = 1;
                }
            }
            // don't update position while exploding
            if (this.sprite.props.exploding) {
                return;
            }
            // update position
            if (move_left) {
                this.sprite.x = Math.max(this.sprite.x - this.sprite.props.speed, min_x);
            } else if (move_right) {
                this.sprite.x = Math.min(this.sprite.x + this.sprite.props.speed, canvas_width - obj_width);
            } 
            // add bullet
            if (shoot && !this.sprite.props.invincible) {
            
                this.sprite.props.last_shot_frame = frame_number;
                this.sprite.props.last_shot_time = Date.now();
                fire_bullet(this.bullets_group, this.sprite.x, this.sprite.y - 50, -1, SHIP_BULLET_SPEED);
                sound.play();
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
        if (!(this.anims.get('enemy' + e + g + '_move'))) {
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
        }
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
            enemy.score = 10 + 10 * (num_rows[0] + num_rows[1] + num_rows[2] - 1 - enemy.grid_row);
            enemy.hit = false;
            enemies.add(enemy);
        }
    }
    // align group enemies in a grid
    Phaser.Actions.GridAlign(enemies.getChildren(), 
    { width: num_horizontal, height: 5, cellWidth: 60, cellHeight: 50, position: Phaser.Display.Align.CENTER, x: x, y: y });
    // set initial velocity for group
    Phaser.Actions.Call(enemies.getChildren(), function(e) {
        e.setVelocityX(-horizontal_speed);
        e.setVelocityY(5);
    })
    // store number of columns in the grid
    var children = enemies.getChildren();
    enemies.num_columns = num_horizontal;
    enemies.num_valid_columns = enemies.num_columns;
    enemies.num_rows = children[children.length - 1].grid_row;

    // create bullets pool
    var bullets = this.create_bullets_pool(30, bullet_image_id, true);
    var sound = this.custom_sounds.fire_enemy;

    // create timer to fire enemy bullets
    enemies.fire_timer = this.time.addEvent({ delay: Phaser.Math.Between(700, 1000), loop: true, 
                                              callback: () => { 
                                                    fire_enemy_bullet(enemies, bullets);
                                              } });
    enemies.timer_changes = 0;

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

            // change timer depending on how many valid columns are left
            if (this.enemies_group.num_valid_columns <= 0.6 * this.enemies_group.num_columns &&
                this.enemies_group.timer_changes == 0) {
                this.enemies_group.fire_timer.reset({ delay: Phaser.Math.Between(1000, 1500), loop: true, 
                                              callback: () => { 
                                                    fire_enemy_bullet(enemies, bullets);
                                              } });
                this.enemies_group.timer_changes += 1;
            }
            else if (this.enemies_group.num_valid_columns <= 0.2 * this.enemies_group.num_columns &&
                this.enemies_group.timer_changes == 1) {
                this.enemies_group.fire_timer.reset({ delay: Phaser.Math.Between(1500, 2000), loop: true, 
                                              callback: () => { 
                                                    fire_enemy_bullet(enemies, bullets);
                                              } });
                this.enemies_group.timer_changes += 1;
            }
        },
    };
}
