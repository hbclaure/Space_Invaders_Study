#!/usr/bin/env python
import sqlite3
import csv
import os
import sys

DATABASE = '/Users/jamielarge/space_invaders_web/space_invaders/db/game_logs.db'

# Get the player's ID from the user
player_id = input('Enter ID of player whose games you would like to export: ')

# Make the folder to contain this player's .csv files
log_folder = 'logs/' + player_id + '/'
if not os.path.exists(log_folder):
    os.makedirs(log_folder)
else:
	sys.exit('Error: game log folder already exists for this player')

con = sqlite3.connect(DATABASE)
cur = con.cursor()

cur.execute('SELECT id, date, mode, round FROM Games WHERE player_id = ?', [player_id])
games_data = cur.fetchall()

with open(log_folder + 'games.csv', 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['id', 'date', 'mode', 'round'])
    writer.writerows(games_data)

with open(log_folder + 'frames.csv', 'w') as frames_f, \
     open(log_folder + 'enemies.csv', 'w') as enemies_f, \
     open(log_folder + 'bullets.csv', 'w') as bullets_f:	
	frames_writer = csv.writer(frames_f)
	frames_writer.writerow(['id', 'game_id', 'frame_number', 'player_position', 'player_lives', 
		                    'player_score', 'ai_position', 'ai_lives', 'ai_score'])

	enemies_writer = csv.writer(enemies_f)
	enemies_writer.writerow(['id', 'frame_id', 'side', 'x', 'y'])

	bullets_writer = csv.writer(bullets_f)
	bullets_writer.writerow(['id', 'frame_id', 'type', 'x', 'y'])

	for game in games_data:
		game_id = game[0]
		cur.execute('SELECT * FROM Frames WHERE game_id = ?', [game_id])
		frames_data = cur.fetchall()
		frames_writer.writerows(frames_data)

		for frame in frames_data:
			frame_id = frame[0]
			cur.execute('SELECT * FROM Enemies WHERE frame_id = ?', [frame_id])
			enemies_data = cur.fetchall()
			enemies_writer.writerows(enemies_data)
			cur.execute('SELECT * FROM Bullets WHERE frame_id = ?', [frame_id])
			bullets_data = cur.fetchall()
			bullets_writer.writerows(bullets_data)
			

