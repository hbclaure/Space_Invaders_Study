// --- First Intermediate Screen of the game (after practice) ---
var intermediate_scene_1 = new Phaser.Scene('intermediate_scene_1');

// load fonts
intermediate_scene_1.preload = function() {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
}

intermediate_scene_1.create = function() {
    var game_name = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'SPACE INVADERS', 50).setOrigin(0.5);
    var instructions = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Use arrow keys to move, \npress spacebar to fire', 25).setOrigin(0.5).setCenterAlign();
    var start_uncoop = this.add.bitmapText(400, 400, 'PressStart2P_Green', 'Press spacebar to begin first round', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

intermediate_scene_1.update = function() {
    // begin cooperative game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        mode = 0;
        this.scene.start('game_scene');
    }
}

// --- Second Intermediate Screen of the game (after cooperative game) ---
var intermediate_scene_2 = new Phaser.Scene('intermediate_scene_2');

// load fonts
intermediate_scene_2.preload = function() {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_Red', 'assets/fonts/PressStart2P_Red/font.png', 'assets/fonts/PressStart2P_Red/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
}

intermediate_scene_2.create = function() {
    // if the player lost the previous round
    if (gameover) {
        var gameover_text = this.add.bitmapText(400, 100, 'PressStart2P_Green', 'GAME OVER', 50).setOrigin(0.5);
    }
    // if the player beat the previous round
    else {
        var victory_text = this.add.bitmapText(400, 100, 'PressStart2P_Green', 'VICTORY!', 50).setOrigin(0.5);
    }

    var player_score = this.add.bitmapText(400, 200, 'PressStart2P_White', 'Player Final Score: ' + player_ship.sprite.props.score, 20).setOrigin(0.5);
    var ai_score = this.add.bitmapText(400, 300, 'PressStart2P_White', 'AI Final Score: ' + ai_ship.sprite.props.score, 20).setOrigin(0.5);
    var start_uncoop = this.add.bitmapText(400, 400, 'PressStart2P_Orange', 'Press spacebar to begin second round', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

intermediate_scene_2.update = function() {
    // begin uncooperative game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        mode = 1;
        this.scene.start('game_scene');
    }
}