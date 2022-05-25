#!/usr/bin/env python3
import csv
import glob
from os import path
import sys

# Get the ID of the player(s) whose data you want to analyze
user_input = input('Type ID of player whose games you would like to analyze (or press enter to analyze all players\' logs): ')

log_paths = []

if (user_input == ''):
	log_paths = glob.glob('logs/*')
else:
	log_paths.append('logs/' + user_input);
	if (path.exists(log_paths[0]) == False):
		sys.exit('No logs for that player exist. Try using export_db.py first.')


# Analyze the logs of each player specified
for log_path in log_paths:
	final_string = ''  # The final string that will be written to a file for this player
	
	# open all the .csv files of this player
	with open(log_path + '/games.csv', newline='') as games_file, \
	     open(log_path + '/frames.csv', newline='') as frames_file, \
	     open(log_path + '/events.csv', newline='') as events_file:
		games_reader = csv.DictReader(games_file)
		frames_reader = csv.DictReader(frames_file)
		events_reader = csv.DictReader(events_file)
		
		games_counter = 1

		frames = []
		for frame in frames_reader:
			frames.append(frame)
		
		events = []
		for event in events_reader:
			events.append(event)

		frame_counter = 0
		event_counter = 0

		for game in games_reader:
			game_time = 0
			player_left_kills = 0
			player_right_kills = 0
			ai_left_kills = 0
			ai_right_kills = 0
			
			# Go through each frame that is in this game
			while (frame_counter < len(frames) and frames[frame_counter]['game_id'] == game['id']):
				# Go through every event that is in this frame
				while (event_counter < len(events) and events[event_counter]['frame_id'] == frames[frame_counter]['id']):
					# Update the player's left/right kills
					if (events[event_counter]['killer'] == 'PLAYER'):
						if (events[event_counter]['killed'] == 'LEFT'):
							player_left_kills += 1
						else:
							player_right_kills += 1

					# Update the AI's left/right kills
					elif (events[event_counter]['killer'] == 'AI'):
						if (events[event_counter]['killed'] == 'LEFT'):
							ai_left_kills += 1
						else:
							ai_right_kills += 1

					event_counter += 1

				game_time += 1
				frame_counter += 1


			gamemode = 'Practice' if game['mode'] == '0' else 'Cooperative' if game['mode'] == '1' else 'Uncooperative'

			# Write the information for this game
			game_string = 'GAME ' + str(games_counter) + ' FOR PLAYER ' + log_path[5:] + ':\n\n' \
						+ 'Mode: ' + gamemode + '\n' \
						+ 'Round: ' + str(int(game['round']) + 1) + '\n' \
						+ 'Total time: ' + str(game_time) + ' frames, or ~' + str(round(game_time / 60, 1)) +'s\n' \
						+ 'Player deaths: ' + str(3 - int(frames[frame_counter - 1]['player_lives'])) + '\n' \
						+ 'Player score: ' + frames[frame_counter - 1]['player_score'] + '\n'

			if (game['mode'] == '0'):
				game_string += 'Player killed ' + str(player_left_kills) + ' enemies\n'
			else:
				game_string += 'Player killed ' + str(player_left_kills) + ' enemies on the left and ' \
				                                + str(player_right_kills) + ' enemies on the right\n' \
				             + 'AI deaths: ' + str(3 - int(frames[frame_counter - 1]['ai_lives'])) + '\n' \
				             + 'AI score: ' + frames[frame_counter - 1]['ai_score'] + '\n' \
				             + 'AI killed ' + str(ai_left_kills) + ' enemies on the left and ' \
				                            + str(ai_right_kills) + ' enemies on the right\n'

			final_string += game_string + '\n\n\n\n'
			games_counter += 1

	with open(log_path + '/report.txt', 'w') as report_file:
		print(final_string[:-5])
		report_file.write(final_string[:-5])