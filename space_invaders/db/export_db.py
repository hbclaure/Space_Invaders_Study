#!/usr/bin/env python3
import sqlite3
import os
import subprocess

DATABASE = '/Users/jamielarge/space_invaders_web/space_invaders/db/game_logs.db'

con = sqlite3.connect(DATABASE)
cur = con.cursor()

# Get the player's ID from the user
user_input = input('Type ID of player whose games you would like to export (or press enter to export all players\' logs): ')

player_ids = []

if (user_input == ''):
  cur.execute('SELECT player_id FROM Games')
  player_ids = list(dict.fromkeys(list(map(lambda x: x[0], cur.fetchall()))))

else:
  player_ids.append(user_input)

for player_id in player_ids:
  print('Creating file for player ' + player_id + '...')
  # Make the folder to contain this player's .csv files
  log_folder = 'logs/' + player_id + '/'
  if not os.path.exists(log_folder):
      os.makedirs(log_folder)

  # Create a .csv of all the games for this player
  subprocess.call(["sqlite3", "game_logs.db", 
    ".headers on", ".mode csv", ".output " + log_folder + "games.csv",
    "SELECT id, date, mode, round FROM Games WHERE player_id = \'" + player_id + "\';", ".quit "])

  # Get a list of all the game IDs
  cur.execute('SELECT id, date, mode, round FROM Games WHERE player_id = ?', [player_id])
  games_data = tuple(map(lambda x: x[0], cur.fetchall()))
  games_data = '(' + str(games_data[0]) + ')' if len(games_data) == 1 else str(games_data)

  # Create a .csv of all the frames for this player
  subprocess.call(["sqlite3", "game_logs.db", 
    ".headers on", ".mode csv", ".output " + log_folder + "frames.csv",
    "SELECT * FROM Frames WHERE game_id IN " + games_data + ";", ".quit "])

  # Get a list of all the frame IDs
  cur.execute("SELECT * FROM Frames WHERE game_id IN " + games_data + ";")
  frames_data = tuple(map(lambda x: x[0], cur.fetchall()))
  frames_data = '(' + str(frames_data[0]) + ')' if len(frames_data) == 1 else str(frames_data)

  # Create a .csv of all the enemies for this player
  subprocess.call(["sqlite3", "game_logs.db", 
    ".headers on", ".mode csv", ".output " + log_folder + "enemies.csv",
    "SELECT * FROM Enemies WHERE frame_id IN " + frames_data + ";", ".quit "])

  # Create a .csv of all the bullets for this player
  subprocess.call(["sqlite3", "game_logs.db", 
    ".headers on", ".mode csv", ".output " + log_folder + "bullets.csv",
    "SELECT * FROM Bullets WHERE frame_id IN " + frames_data + ";", ".quit "])

  # Create a .csv of all the events for this player
  subprocess.call(["sqlite3", "game_logs.db", 
    ".headers on", ".mode csv", ".output " + log_folder + "events.csv",
    "SELECT * FROM Events WHERE frame_id IN " + frames_data + ";", ".quit "])