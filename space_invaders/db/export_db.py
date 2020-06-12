#!/usr/bin/env python3
import sqlite3
import os
import sys
import subprocess

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

# Create a .csv of all the games for this player
subprocess.call(["sqlite3", "game_logs.db", 
  ".mode csv", ".output " + log_folder + "games.csv",
  "SELECT id, date, mode, round FROM Games WHERE player_id = \'" + player_id + "\';", ".quit "])

# Get a list of all the game IDs
cur.execute('SELECT id, date, mode, round FROM Games WHERE player_id = ?', [player_id])
games_data = str(tuple(map(lambda x: x[0], cur.fetchall())))

# Create a .csv of all the frames for this player
subprocess.call(["sqlite3", "game_logs.db", 
  ".mode csv", ".output " + log_folder + "frames.csv",
  "SELECT * FROM Frames WHERE game_id IN " + games_data + ";", ".quit "])

# Get a list of all the frame IDs
cur.execute("SELECT * FROM Frames WHERE game_id IN " + games_data + ";")
frames_data = str(tuple(map(lambda x: x[0], cur.fetchall())))

# Create a .csv of all the enemies for this player
subprocess.call(["sqlite3", "game_logs.db", 
  ".mode csv", ".output " + log_folder + "enemies.csv",
  "SELECT * FROM Enemies WHERE frame_id IN " + frames_data + ";", ".quit "])

# Create a .csv of all the bullets for this player
subprocess.call(["sqlite3", "game_logs.db", 
  ".mode csv", ".output " + log_folder + "bullets.csv",
  "SELECT * FROM Bullets WHERE frame_id IN " + frames_data + ";", ".quit "])
