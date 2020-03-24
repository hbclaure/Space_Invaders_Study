// Space Invaders game built with Phaser 3
// This game is inspired by Lee Robinson's Space Invaders, https://leerob.io/blog/space-invaders-with-python
// And an adaptation by Simon Mendelsohn, https://zoo.cs.yale.edu/classes/cs490/19-20a/mendelsohn.simon.sjm225

// --- helper functions ---

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
function create_ship(image_id="ship", x = 200, y = 540, speed = 5, bullet_image_id = "laser") {
            
    canvas_width = this.sys.canvas.width;
    canvas_height = this.sys.canvas.height;
    imageobj = this.add.image(canvas_width/4, canvas_height - 10, image_id).setOrigin(0.5, 1.0); 
    imageobj.props = {} // add properties object to ship object
    imageobj.props.speed = speed;
    obj_width = imageobj.displayWidth;     

    bullets = this.physics.add.group({ // bullets pool. They become visible when a bullet is fired
        key: bullet_image_id,
        repeat: 30,
        visible: false,
        active: false,
        collideWorldBounds: true,
    }); 

    return {
        imageobj: imageobj,                                     //!< image object for the ship
        bullets: bullets,                                       //!< bullets shot by the ship
        update(move_left, move_right, shoot)                    //!< update the ship state based on requested actions   
        {                
            // update position
            if (move_left) {
                this.imageobj.x = Math.max(this.imageobj.x - this.imageobj.props.speed, obj_width);
            } else if (move_right) {
                this.imageobj.x = Math.min(this.imageobj.x + this.imageobj.props.speed, canvas_width - obj_width);
            } 
            // add bullet
            if (shoot) {
                let bullet = this.bullets.get(this.imageobj.x, this.imageobj.y - 50);
                if (bullet) {
                    // set bullet position and speed
                    bullet.body.originX = 0.5;
                    bullet.body.originY = 1.0;
                    bullet.body.velocity.y = -500;
                    bullet.body.allowGravity = true;
                    // make bullet visible
                    bullet.setActive(true);
                    bullet.setVisible(true);
                    // setup callback to hide bullet when it reaches the end of the screen
                    bullet.body.onWorldBounds = true;                    
                    bullet.body.world.on('worldbounds', function(body) {
                        if (body.gameObject == bullet) {
                            bullet.setVisible(false);
                            bullet.setActive(false);
                        }
                    })

                }
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
            gravity: { y: 100 }
        }
    },
    scene: {
        preload: preload,
        create: create,
        update: update,
        extend: {
            create_ship: create_ship,
        }
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

    ship1 = this.create_ship(image_id="ship", x = this.sys.canvas.width / 4, y = 540);

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
        enemy1 = this.physics.add.sprite(400, 300, 'enemy1_1').play('enemy1_move');
        enemy1.setOrigin(0.5, 1.0);
        enemy1.displayWidth = 50;
        enemy1.scaleY = enemy1.scaleX;
        // enemy1.setDisplaySize(30,30)
        enemies_group1.add(enemy1);
    }
    // add cyan enemies
    for (var i=0; i<5; i++) {
        enemy2 = this.physics.add.sprite(400, 300, 'enemy2_1').setScale(0.5).play('enemy2_move');
        enemy2.setOrigin(0.5, 1.0);
        enemy2.displayWidth = 50;
        enemy2.scaleY = enemy2.scaleX;
        // enemy2.setDisplaySize(30,30)
        enemies_group1.add(enemy2);
    }
    // add green enemies
    for (var i=0; i<5; i++) {
        enemy3 = this.physics.add.sprite(400, 300, 'enemy3_1').setScale(0.5).play('enemy3_move');
        enemy3.setOrigin(0.5, 1.0);
        enemy3.displayWidth = 50;
        enemy3.scaleY = enemy3.scaleX;
        // enemy2.setDisplaySize(30,30)
        enemies_group1.add(enemy3);
    }
    // align in a grid
    Phaser.Actions.GridAlign(enemies_group1.getChildren(), 
    { width: 5, height: 3, cellWidth: 50, cellHeight: 50, position: Phaser.Display.Align.CENTER, x: 100, y: 0 });

    this.physics.add.collider(enemies_group1, ship1.bullets, (enemy, bullet) => {
        // destroy the enemy
        enemy.destroy();
        // move bullet to the edge.. it will become not visible automatically
        bullet.body.x = this.sys.canvas.width;
        bullet.body.y = this.sys.canvas.height;
    })
}

function update ()
{
    ship1.update(cursors.left.isDown, cursors.right.isDown, this.input.keyboard.checkDown(space_key, 500));
}