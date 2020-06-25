/**
 * Preload assets for the game
 */
function preload ()
{
    // load images
    this.load.setBaseURL(static_url + '/');
    this.load.image('ship', 'assets/images/ship.png');
    this.load.image('avery', 'assets/images/avery.png');
    this.load.image('jordan', 'assets/images/jordan.png');
    this.load.image('mystery', 'assets/images/mystery.png');

    this.load.image('enemy1a_1', 'assets/images/enemy1_1.png');
    this.load.image('enemy1a_2', 'assets/images/enemy1_2.png');
    this.load.image('enemy2a_1', 'assets/images/enemy2_1.png');
    this.load.image('enemy2a_2', 'assets/images/enemy2_2.png');
    this.load.image('enemy3a_1', 'assets/images/enemy3_1.png');
    this.load.image('enemy3a_2', 'assets/images/enemy3_2.png');

    this.load.image('enemy1b_1', 'assets/images/enemy1_1.png');
    this.load.image('enemy1b_2', 'assets/images/enemy1_2.png');
    this.load.image('enemy2b_1', 'assets/images/enemy2_1.png');
    this.load.image('enemy2b_2', 'assets/images/enemy2_2.png');
    this.load.image('enemy3b_1', 'assets/images/enemy3_1.png');
    this.load.image('enemy3b_2', 'assets/images/enemy3_2.png');

    this.load.image('enemy1p_1', 'assets/images/enemy1_1.png');
    this.load.image('enemy1p_2', 'assets/images/enemy1_2.png');
    this.load.image('enemy2p_1', 'assets/images/enemy2_1.png');
    this.load.image('enemy2p_2', 'assets/images/enemy2_2.png');
    this.load.image('enemy3p_1', 'assets/images/enemy3_1.png');
    this.load.image('enemy3p_2', 'assets/images/enemy3_2.png');

    this.load.image('explosionblue', 'assets/images/explosionblue.png');
    this.load.image('explosiongreen', 'assets/images/explosiongreen.png');
    this.load.image('explosionpurple', 'assets/images/explosionpurple.png');
    this.load.image('shipexplosion', 'assets/images/shipexplosion.png');
    this.load.image('averyexplosion', 'assets/images/averyexplosion.png');
    this.load.image('jordanexplosion', 'assets/images/jordanexplosion.png');
    this.load.image('laser', 'assets/images/laser.png');
    this.load.image('enemylaser', 'assets/images/enemylaser.png');

    // load audio
    this.load.audio("audio_explosion", "assets/sounds/shipexplosion.wav");
    this.load.audio("audio_fire_ship", "assets/sounds/shoot.wav");

    // load fonts
    this.load.bitmapFont('PressStart2P_Gray', 'assets/fonts/PressStart2P_Gray/font.png', 'assets/fonts/PressStart2P_Gray/font.fnt');
    this.load.bitmapFont('PressStart2P_Orange', 'assets/fonts/PressStart2P_Orange/font.png', 'assets/fonts/PressStart2P_Orange/font.fnt');
    this.load.bitmapFont('PressStart2P_Purple', 'assets/fonts/PressStart2P_Purple/font.png', 'assets/fonts/PressStart2P_Purple/font.fnt');
}