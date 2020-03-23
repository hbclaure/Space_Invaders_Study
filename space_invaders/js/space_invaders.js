// Space Invaders game built with Phaser 3
// This game is inspired by Lee Robinson's Space Invaders, https://leerob.io/blog/space-invaders-with-python
// And an adaptation by Simon Mendelsohn, https://zoo.cs.yale.edu/classes/cs490/19-20a/mendelsohn.simon.sjm225

// --- helper functions ---


/**
 * Create bullet
 * @param {Phaser.Scene} scene_instance Scene instance in which to create the ship 
 * @param {string} image_id Image ID for the ship
 * @param {int} x horizontal coordinate for the bullet (fixed)
 * @param {int} y vertical coordinate for the bullet (moves horizontally)
 * @param {int} direction -1 for up, 1 for down movement
 * @param {int} speed bullet speed (absolute value)
 * @param {int} min_y minimum y position
 * @param {int} max_y maximum y position
 */
function create_bullet(scene_instance, image_id, x, y, direction, speed = 10, min_y = 15, max_y = 600) {
    return {
        imageobj: scene_instance.add.image(x, y, image_id),
        direction: direction,
        speed: speed,
        x: x,
        y: y,
        min_y: min_y,
        max_y: max_y,
        killed: false,
        update: function() {
            console.log(this.y);
            if (!this.killed) {
                this.y = this.y + this.speed * this.direction;
                if (this.y < this.min_y || this.y > this.max_y) {
                    console.log("killed bullet");
                    this.killed = true;
                    this.imageobj.destroy();
                } else {
                    this.imageobj.y = this.y;
                }
            }
        },
    };
}

/**
 * Create ship object
 * @param {Phaser.Scene} scene_instance Scene instance in which to create the ship 
 * @param {string} image_id Image ID for the ship
 * @param {int} x horizontal coordinate for the ship (moves horizontally)
 * @param {int} y vertical coordinate for the ship (fixed)
 * @param {int} speed speed for the ship
 * @param {int} min_x minimum allowed horizontal position for the ship
 * @param {int} max_x maximum allowed horizontal position for the ship
 * @param {int} shooting_direction shooting direction (-1 means up, 1 means down)
 * @param {String} bullet_image_id image id for the bullet
 */
function create_ship(scene_instance, image_id="ship", x = 200, y = 540, speed = 5, min_x = 50, max_x = 750, 
                     shooting_direction = -1, bullet_image_id="laser") {
    return {
        scene_instance: scene_instance,                         //!< scene instance
        imageobj: scene_instance.add.image(50, 50, image_id),   //!< image object for the ship
        speed: speed,                                           //!< moving speed for the ship
        x: x,                                                   //!< current x position
        y: y,                                                   //!< current y position
        min_x: min_x,                                           //!< minimum x position
        max_x: max_x,                                           //!< maximum x position
        bullets: [],                                            //!< bullets shot by the ship
        shooting_direction: shooting_direction,                 //!< bullet direction
        bullet_image_id: bullet_image_id,                       //!< bullet image id
        update: function(move_left, move_right, shoot)          //!< update the ship state based on requested actions
        {        
            // update position
            if (move_left) {
                left_move = this.x - this.speed;
                if (left_move > this.min_x) 
                    this.x = left_move;
            } else if (move_right) {
                right_move = this.x + this.speed;
                if (right_move < this.max_x) 
                    this.x += this.speed;
            } 
            // update ship image based on new position
            this.imageobj.x = this.x;
            this.imageobj.y = this.y;
            // add bullet
            if (shoot) {
                bullet = create_bullet(this.scene_instance, bullet_image_id, 
                                       this.x, this.y + (30*this.shooting_direction), this.shooting_direction)
                this.bullets.push(bullet)
                console.log(this.bullets.length)
            }
            // update bullets
            for(var i=0; i<this.bullets.length; i++) {
                this.bullets[i].update()
            }
        },
    };
}


// --- actual game ---

var config = {
    type: Phaser.AUTO,
    width: 800,
    height: 600,
    physics: {
        default: 'arcade',
        arcade: {
            gravity: { y: 200 }
        }
    },
    scene: {
        preload: preload,
        create: create,
        update: update
    },
    fps: 30,
};

var game = new Phaser.Game(config);
var cursors;                            // keyboard access
var space_key;                          // space key
var ship1;                              // ship1
var enemies_group1;

function preload ()
{

    // load imagess
    // this.load.setBaseURL('..');
    this.load.image('ship', 'assets/images/ship.png');
    this.load.image('avery', 'assets/images/avery.png');
    this.load.image('jordan', 'assets/images/jordan.png');
    this.load.image('mystery', 'assets/images/mystery.png');
    this.load.image('enemy1_1', 'assets/images/enemy1_1.png');
    this.load.image('enemy1_2', 'assets/images/enemy1_2.png');
    this.load.image('enemy2_1', 'assets/images/enemy2_1.png');
    this.load.image('enemy2_2', 'assets/images/enemy2_2.png');
    this.load.image('enemy3_1', 'assets/images/enemy3_1.png');
    this.load.image('enemy3_2', 'assets/images/enemy3_2.png');
    this.load.image('explosionblue', 'assets/images/explosionblue.png');
    this.load.image('explosiongreen', 'assets/images/explosiongreen.png');
    this.load.image('explosionpurple', 'assets/images/explosionpurple.png');
    this.load.image('laser', 'assets/images/laser.png');
    this.load.image('enemylaser', 'assets/images/enemylaser.png');

}

function create ()
{
    cursors = this.input.keyboard.createCursorKeys();
    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);

    ship1 = create_ship(this);

    this.anims.create({
        key: 'enemy1_move',
        frames: [
            { key: 'enemy1_1' },
            { key: 'enemy1_2' },
        ],
        frameRate: 2,
        repeat: -1
    });

    this.anims.create({
        key: 'enemy2_move',
        frames: [
            { key: 'enemy2_1' },
            { key: 'enemy2_2' },
        ],
        frameRate: 2,
        repeat: -1
    });

    this.anims.create({
        key: 'enemy3_move',
        frames: [
            { key: 'enemy3_1' },
            { key: 'enemy3_2' },
        ],
        frameRate: 2,
        repeat: -1
    });

    this.anims.create({
        key: 'enemy1_explote',
        frames: [
            { key: 'enemy1_1' },
            { key: 'explosionpurple' },
        ],
        frameRate: 2,
        repeat: 0,
        hideOnComplete: true,
    });

    enemies_group1 = this.add.group();
    // add purple enemies
    for (var i=0; i<5; i++) {
        enemy1 = this.add.sprite(400, 300, 'enemy1_1').play('enemy1_move');
        // enemy1.setDisplaySize(30,30)
        enemies_group1.add(enemy1);
    }
    // add cyan enemies
    for (var i=0; i<5; i++) {
        enemy2 = this.add.sprite(400, 300, 'enemy2_1').play('enemy2_move');
        // enemy2.setDisplaySize(30,30)
        enemies_group1.add(enemy2);
    }
    // align in a grid
    Phaser.Actions.GridAlign(enemies_group1.getChildren(), 
    { width: 5, height: 2, cellWidth: 200, cellHeight: 200, position: Phaser.Display.Align.TOP_LEFT, x: 0, y: 0 });

    
}

function update ()
{
    ship1.update(cursors.left.isDown, cursors.right.isDown, this.input.keyboard.checkDown(space_key, 500));
}