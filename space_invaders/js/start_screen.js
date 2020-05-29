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
    var start_practice = this.add.bitmapText(400, 400, 'PressStart2P_Green', 'Press spacebar to begin practice round', 20).setOrigin(0.5);

    space_key = this.input.keyboard.addKey(Phaser.Input.Keyboard.KeyCodes.SPACE);
}

start_scene.update = function() {
    // begin practice game when the player presses spacebar
    if (this.input.keyboard.checkDown(space_key, 500)) {
        this.scene.start('practice_scene');
    }
}