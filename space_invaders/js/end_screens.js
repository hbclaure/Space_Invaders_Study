// --- Game Over Screen ---
var gameover_scene = new Phaser.Scene('gameover_scene');

// load fonts
gameover_scene.preload = function () {
    this.load.bitmapFont('PressStart2P_Red', 'assets/fonts/PressStart2P_Red/font.png', 'assets/fonts/PressStart2P_Red/font.fnt');
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
}

// display Game Over and final scores
gameover_scene.create = function() {
    var gameover_text = this.add.bitmapText(400, 175, 'PressStart2P_Red', 'GAME OVER', 50).setOrigin(0.5);
    var player_score = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Player Final Score: ' + player_ship.sprite.props.score, 20).setOrigin(0.5);
    var ai_score = this.add.bitmapText(400, 400, 'PressStart2P_White', 'AI Final Score: ' + ai_ship.sprite.props.score, 20).setOrigin(0.5);
}


// --- Victory Screen ---
var victory_scene = new Phaser.Scene('victory_scene');

// load fonts
victory_scene.preload = function () {
    this.load.bitmapFont('PressStart2P_White', 'assets/fonts/PressStart2P_White/font.png', 'assets/fonts/PressStart2P_White/font.fnt');
    this.load.bitmapFont('PressStart2P_Green', 'assets/fonts/PressStart2P_Green/font.png', 'assets/fonts/PressStart2P_Green/font.fnt');
}

// display Victory! and final scores
victory_scene.create = function() {
    var victory_text = this.add.bitmapText(400, 175, 'PressStart2P_Green', 'VICTORY!', 50).setOrigin(0.5);
    var player_score = this.add.bitmapText(400, 300, 'PressStart2P_White', 'Player Final Score: ' + player_ship.sprite.props.score, 20).setOrigin(0.5);
    var ai_score = this.add.bitmapText(400, 400, 'PressStart2P_White', 'AI Final Score: ' + ai_ship.sprite.props.score, 20).setOrigin(0.5);
}
