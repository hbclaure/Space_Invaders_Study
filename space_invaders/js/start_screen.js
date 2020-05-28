// --- Start Screen of the game ---
var start_scene = new Phaser.Scene('start_scene');

// load fonts
start_scene.preload = function() {
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
    this.load.bitmapFont('PressStart2P_Red', 'assets/fonts/PressStart2P_Red/font.png', 'assets/fonts/PressStart2P_Red/font.fnt');
}

start_scene.create = function() {
    var game_name = this.add.bitmapText(400, 175, 'PressStart2P_Orange', 'SPACE INVADERS', 50).setOrigin(0.5);
    var instructions = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Use arrow keys to move, press spacebar to fire', 15).setOrigin(0.5);
    var start_uncoop = this.add.bitmapText(400, 400, 'PressStart2P_Red', 'Press spacebar to begin', 20).setOrigin(0.5);
    var start_uncoop_2 = this.add.bitmapText(400, 425, 'PressStart2P_Red', 'uncooperative game', 20).setOrigin(0.5);
    var start_coop = this.add.bitmapText(400, 500, 'PressStart2P_Green', 'Press enter to begin cooperative game', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
    enter_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.ENTER);
}

start_scene.update = function() {
    // begin uncooperative game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        mode = 1;
        this.scene.switch('game_scene');
    }

    // begin cooperative game when the player presses enter
    if (this.input.keyboard.checkDown(enter_key, 500)) {
        mode = 0;
        this.scene.switch('game_scene');
    }
}