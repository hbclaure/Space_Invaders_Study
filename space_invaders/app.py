from flask import Flask, g, request, render_template, Response
import sqlite3
import json

app = Flask(__name__)

DATABASE = '/Users/jamielarge/space_invaders_web/space_invaders/db/game_logs.db'

# get the database connection
def get_db():
	db = getattr(g, '_database', None)
	if db is None:
		db = g._database = sqlite3.connect(DATABASE)
	return db


# close the database connection
@app.teardown_appcontext
def close_connection(exception):
	db = getattr(g, '_database', None)
	if db is not None:
		db.close()

@app.route('/')
def run_game():
	return render_template('index.html')


@app.route('/log', methods=['GET', 'POST'])
def log_games():
	# open the connection and get a cursor
	con = get_db();
	cur = con.cursor();

	logs = json.loads(request.data);

	# Go through each game log
	for log in logs:
		# Log the actual game and collect the id
		cur.execute('INSERT INTO Games(player_id, date, round, mode) VALUES(?, ?, ?, ?)', 
			        (log['player_id'], log['date'], log['round'], log['mode']))

		game_id = cur.lastrowid

		# Keep track of the event that we're checking
		current_event = 0;

		# Go through every frame of the game
		for frame in log['frames']:
			# log the actual frame and collect the id
			cur.execute('''INSERT INTO Frames(game_id, frame_number, player_position, player_lives, 
				        player_score, ai_position, ai_lives, ai_score) VALUES(?,?,?,?,?,?,?,?)''', 
		 		        (game_id, frame['frame_number'], frame['player_position'], frame['player_lives'], 
		 		         frame['player_score'], frame['ai_position'], frame['ai_lives'], frame['ai_score']))
			frame_id = cur.lastrowid;

			# log the player bullet if it exists
			if (len(frame['player_bullet_position']) > 0):
				cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Player\',?,?,?)',
				   		    (frame_id, frame['player_bullet_position'][0], frame['player_bullet_position'][1]))

			# log the ai bullet if it exists
			if (len(frame['ai_bullet_position']) > 0):
				cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'AI\',?,?,?)',
				   		    (frame_id, frame['ai_bullet_position'][0], frame['ai_bullet_position'][1]))

			# log every enemy on the left
			for enemy in frame['enemies_left_positions']:
				cur.execute('INSERT INTO Enemies(side, frame_id, x, y) VALUES(\'Left\',?,?,?)',
					       (frame_id, enemy[0], enemy[1]))

			# log every enemy bullet on the left
			for bullet in frame['bullets_left_positions']:
				cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Left\',?,?,?)',
				   		    (frame_id, bullet[0], bullet[1]))

			# log every enemy on the right
			for enemy in frame['enemies_right_positions']:
				cur.execute('INSERT INTO Enemies(side, frame_id, x, y) VALUES(\'Right\',?,?,?)',
					       (frame_id, enemy[0], enemy[1]))

			# log every enemy bullet on the right
			for bullet in frame['bullets_right_positions']:
				cur.execute('INSERT INTO Bullets(type, frame_id, x, y) VALUES(\'Right\',?,?,?)',
				   		    (frame_id, bullet[0], bullet[1]))

			# log any events that occurred in this frame
			while (current_event < len(log['events']) and log['events'][current_event]['frame'] == frame['frame_number']):
				cur.execute('INSERT INTO Events(frame_id, killer, killed) VALUES(?,?,?)',
					        (frame_id, log['events'][current_event]['killer'], log['events'][current_event]['killed']))
				current_event += 1

	con.commit()
	return Response("{}", status=201, mimetype='application/json')
