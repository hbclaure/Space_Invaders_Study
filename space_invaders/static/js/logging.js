const initSqlJs = require('sql.js');
// var initSqlJs = window.initSqlJs;
const fetch = require("node-fetch");

async function add_game_log(log) {
	const fetched = await fetch("/Users/jamielarge/space_invaders_web/space_invaders/db/game_logs.db");
	const buf = await fetched.arrayBuffer();
	const db = new SQL.Database(new Uint8Array(buf));

	// log the actual game and collect the ID
	await db.run('INSERT INTO Games(player_id, date, round, mode) VALUES(:p, :d, :r, :m)',
				{'p':log.player_id, 'd':log.date.toString(), 'r':log.round, 'm':log.mode});
	var game_id = await db.exec('SELECT last_insert_rowid()');
	console.log(game_id);
}

var TEST_LOG = {player_id: 'booboo', date: 'today', round: 0, mode: 0, frames: []};

add_game_log(TEST_LOG);

// 	db.run('INSERT INTO Games(player_id, date, round, mode) VALUES(?,?,?,?)', 
// 							 [log.id, log.date.toString(), log.round, log.mode], function(err) {
// 		add_frame_log(this.lastID, log.game_log);
// 		if (err) {
// 				console.log(err.message);
// 		}
// 	});
// }

// function add_frame_log(game_id, frames) {
// 	// log every frame
// 	for (var i = 0; i < frames.length; i++) {
// 		var current_frame = frames[i];
// 		// log the frame and collect the ID
// 		db.run('INSERT INTO ' +
// 			   'Frames(game_id, frame_number, player_position, player_lives,' +
// 			   'player_score, ai_position, ai_lives, ai_score) VALUES(?,?,?,?,?,?,?,?)', 
// 			   [game_id, current_frame.frame_number, current_frame.player_position, current_frame.player_lives,
// 			   current_frame.player_score, current_frame.ai_position, current_frame.ai_lives, current_frame.ai_score],
// 			   function(err) {
// 			if (err) {
// 				console.log(err.message);
// 			}
// 		});
			

// 		// log the player bullet if it exists
// 		if (current_frame.player_bullet_position.length > 0) {
// 			db.run('INSERT INTO Bullets(game_id, frame_number, type, x, y) VALUES(?,?, \'Player\',?,?)',
// 			   		[game_id, current_frame.frame_number, current_frame.player_bullet_position[0], current_frame.player_bullet_position[1]], function (err) {
// 			   			if (err) {
// 							console.log(err.message);
// 						}
// 			   		});
// 		}

// 		// log the AI bullet if it exists
// 		if (current_frame.ai_bullet_position.length > 0) {
// 			db.run('INSERT INTO Bullets(game_id, frame_number, type, x, y) VALUES(?,?, \'AI\',?,?)',
// 			   		[game_id, current_frame.frame_number, current_frame.ai_bullet_position[0], current_frame.ai_bullet_position[1]], function (err) {
// 			   			if (err) {
// 							console.log(err.message);
// 						}
// 			   		});
// 		}

// 		// log every enemy bullet on the left
// 		for (var j = 0; j < current_frame.bullets_left_positions.length; j++) {
// 			var current_bullet = current_frame.bullets_left_positions[j];
// 			db.run('INSERT INTO Bullets(game_id, frame_number, type, x, y) VALUES(?,?, \'Left\',?,?)',
// 			   		[game_id, current_frame.frame_number, current_bullet[0], current_bullet[1]], function (err) {
// 			   			if (err) {
// 							console.log(err.message);
// 						}
// 			   		});
// 		}

// 		// log every enemy bullet on the right
// 		for (var j = 0; j < current_frame.bullets_right_positions.length; j++) {
// 			var current_bullet = current_frame.bullets_right_positions[j];
// 			db.run('INSERT INTO Bullets(game_id, frame_number, type, x, y) VALUES(?,?, \'Right\',?,?)',
// 			   		[game_id, current_frame.frame_number, current_bullet[0], current_bullet[1]], function (err) {
// 			   			if (err) {
// 							console.log(err.message);
// 						}
// 			   		});
// 		}

// 		// log every enemy on the left
// 		for (var j = 0; j < current_frame.enemies_left_positions.length; j++) {
// 			var current_enemy = current_frame.enemies_left_positions[j];
// 			db.run('INSERT INTO Enemies(game_id, frame_number, side, x, y) VALUES(?,?, \'Left\',?,?)',
// 			   		[game_id, current_frame.frame_number, current_enemy[0], current_enemy[1]], function (err) {
// 			   			if (err) {
// 							console.log(err.message);
// 						}
// 			   		});
// 		}

// 		// log every enemy on the right
// 		for (var j = 0; j < current_frame.enemies_right_positions.length; j++) {
// 			var current_enemy = current_frame.enemies_right_positions[j];
// 			db.run('INSERT INTO Enemies(game_id, frame_number, side, x, y) VALUES(?,?, \'Right\',?,?)',
// 			   		[game_id, current_frame.frame_number, current_enemy[0], current_enemy[1]], function (err) {
// 			   			if (err) {
// 							console.log(err.message);
// 						}
// 			   		});
// 		}
// 	}
// }