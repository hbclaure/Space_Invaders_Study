// --- Start Screen of the game ---
var start_scene = new Phaser.Scene('start_scene');

// load fonts
start_scene.preload = function() {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
}

start_scene.create = function() {
    var game_name = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'SPACE INVADERS', 50).setOrigin(0.5);
    var instructions = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Use arrow keys to move, \npress spacebar to fire', 25).setOrigin(0.5).setCenterAlign();
    var start = this.add.bitmapText(400, 400, 'PressStart2P_Green', 'Press spacebar to begin', 20).setOrigin(0.5);

    if (mode == -1) {
        start.setText('Press spacebar to begin practice round');
    }

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

start_scene.update = function() {
    // begin practice game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        if (mode == -1) {
            this.scene.start('practice_scene');
        }
        else {
            this.scene.start('game_scene');
        }
    }
}


// --- First Intermediate Screen of the game (after practice) ---
var intermediate_scene = new Phaser.Scene('intermediate_scene');

// load fonts
intermediate_scene.preload = function() {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
}

intermediate_scene.create = function() {
    player_score = player_ship.sprite.props.score;
    ai_score = ai_ship.sprite.props.score;

    var gameover_text = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'ROUND ENDED', 50).setOrigin(0.5);
    var player_text = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Player Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    var ai_text = this.add.bitmapText(400, 400, 'PressStart2P_White', 'AI Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
    var instructions = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Press spacebar to begin second round', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

intermediate_scene.update = function() {
    // begin second game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        this.scene.start('game_scene');
    }
}

// --- Game Over Screen ---
var gameover_scene = new Phaser.Scene('gameover_scene');

// load fonts
gameover_scene.preload = function () {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
}

// display Game Over and final scores
gameover_scene.create = function() {
    player_score += player_ship.sprite.props.score;
    ai_score += ai_ship.sprite.props.score;

    var gameover_text = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'GAME ENDED', 50).setOrigin(0.5);
    var player_text = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Player Final Score: ' + player_score, 20).setOrigin(0.5).setCenterAlign();
    var ai_text = this.add.bitmapText(400, 400, 'PressStart2P_White', 'AI Final Score: ' + ai_score, 20).setOrigin(0.5).setCenterAlign();
}

// --- Game Over Screen ---
var gameover_scene_practice = new Phaser.Scene('gameover_scene_practice');

// load fonts
gameover_scene_practice.preload = function () {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
}

// display Game Over and final scores
gameover_scene_practice.create = function() {
    var gameover_text = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'GAME ENDED', 50).setOrigin(0.5);
    var player_text = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Player Score: ' + player_ship.sprite.props.score, 20).setOrigin(0.5).setCenterAlign();;
}