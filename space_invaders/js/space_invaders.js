// Space Invaders game built with Phaser 3
// This game is inspired by Lee Robinson's Space Invaders, https://leerob.io/blog/space-invaders-with-python
// And an adaptation by Simon Mendelsohn, https://zoo.cs.yale.edu/classes/cs490/19-20a/mendelsohn.simon.sjm225
// Framework for this code was built by Professor Marynel Vázquez, expanded upon by Ananya Parthasarathy as 
// part of a CPSC 490 Senior project, and further expanded upon by Jamie Large.


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
};

var game_scene = {
    preload: preload,
    create: create,
    update: update,
    extend: {
        fire_bullet: fire_bullet,
        create_bullets_pool: create_bullets_pool,
        create_ship: create_ship,
        create_enemies: create_enemies,
    }
};


var game = new Phaser.Game(config);     //!< game object

// Add all of the scenes (see start_scene.js and end_scenes.js)
game.scene.add('start_scene', start_scene);
game.scene.add('intermediate_scene_1', intermediate_scene_1);
game.scene.add('intermediate_scene_2', intermediate_scene_2);
game.scene.add('gameover_scene', gameover_scene);
game.scene.add('game_scene', game_scene);
game.scene.add('practice_scene', practice_scene);


game.scene.start('start_scene');
