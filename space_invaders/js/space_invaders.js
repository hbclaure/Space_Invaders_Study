// Space Invaders game built with Phaser 3
// This game is inspired by Lee Robinson's Space Invaders, https://leerob.io/blog/space-invaders-with-python
// And an adaptation by Simon Mendelsohn, https://zoo.cs.yale.edu/classes/cs490/19-20a/mendelsohn.simon.sjm225

// --- helper functions ---

/**
 * Create ship object
 * @param {Phaser.Scene} scene_instance Scene instance in which to create the ship 
 * @param {string} image_id Image ID for the ship
 */
function create_ship(scene_instance, image_id) {
    return {
        imageobj: scene_instance.add.image(50, 50, image_id),
        direction: -1,
        speed: 5,
        x: 200,
        y: 540,
        min_x: 10,
        max_x: 740,
        update: function(left_cursor, right_cursor) {
            if (left_cursor && this.x > this.min_x) {
                this.x -= this.speed;
            } else if (right_cursor && this.x < this.max_x) {
                this.x += this.speed;
            }
            this.imageobj.x = this.x;
            this.imageobj.y = this.y;
        },
    };
}

// --- actual game ---

var config = {
    type: Phaser.AUTO,
    width: 800,
    height: 600,
    // physics: {
    //     default: 'arcade',
    //     arcade: {
    //         gravity: { y: 200 }
    //     }
    // },
    scene: {
        preload: preload,
        create: create,
        update: update
    }
};

var game = new Phaser.Game(config);
var cursors;                            // keyboard access
var ship1;                              // ship1

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

    ship1 = create_ship(this, "ship");

}

function update ()
{
    ship1.update(cursors.left.isDown, cursors.right.isDown);

}