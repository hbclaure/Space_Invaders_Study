# Information about game_logs.db
*Last updated: 6/10/2021

After a game is completed, the JS client sends data to the Python server (via a Websocket) to log game informaiton in an [SQLite](https://www.sqlite.org/index.html) database. Once the green text at the botom of the "game ended" screen changes from "Loading..." to a valid completion code, the data has been successfully saved to the database.

The game_logs.db consists of 6 tables:
1. [Games](#games)
2. [Frames](#frames)
3. [Actions](#actions)
4. [Bullets](#bullets)
5. [Enemies](#enemies)
6. [Events](#events)


## 1. Games
Each row in the Games table represents one completed round of the game. 

The Games table has 4 columns:
- **id**: the row number; **game_id** for game used to identify the game in [Frames](#frames) table
- **player_id**: the ID passed in via the `id=` URL parameter (default is `UNDEFINED`)
- **date**: timestamp of when game was created
- **mode**: the mode of the game passed in via the `mode=` URL parameter (default is 1)
    - 1 (*default*)= Cooperative Early: agent comes over to help twice before all of own enemies destroyed
    - 2 = Cooperative Late: agent comes over to help after all of own enemies destroyed
    - 3 = Uncooperative: agent stays on right side


## 2. Frames
Each row of the Frames table represents one frame of a game. A frame is one pass through the `update()` function for the game.

The Frames tables has 10 columns:
- **id**: the row number; **frame_id** used in other tables to identigy the corresponding frame (don't use true **frame_number** in other tables because you would need a double lookup with **game_id**)
- **game_id**: matches **id** from [Games](#games) table
- **timestamp**: timestamp for each frame
- **frame_number**: frame_number from game loop
- **player_position**: x-coordinate for player ship
- **player_lives**: number of lives remaining for player
- **player_score**: total points scored up to this frame
- **ai_position**: x-coordinate for agent ship
- **ai_lives**: number of lives remaining for the agent ship
- **ai_score**: total points scored up to this frame
- **frame_sent**: indicator variable if frame was sent via control socket
    - O: not sent
    - 1: sent


## 3. [Actions](#actions)
Each row of the Actions table represents one frame of a game. A frame is one pass through the `update()` function for the game.

The Frames tables has 12 columns:
- **frame_id**: matches **id** from [Frames](#frames) table
- **frame_sent**: indicator variable if frame was sent via control socket
    - O: not sent
    - 1: sent
- **player_left**: indicator variable if player tried to move left (if **player_left** = 1 AND **frame_sent** = 1, player did move left)
    - 0: player did not press left arrow to move left
    - 1: player did press left arrow to move left
- **player_right**: indicator variable if player tried to move right (if **player_right** = 1 AND **frame_sent** = 1, player did move right)
    - 0: player did not press right arrow to move right
    - 1: player did press right arrow to move right
- **player_shoot**: indicator variable if player tried to shoot and game rules would allow
    - 0: player did not press space bar to shoot or game rules did not allow player to shoot (e.g., not enough time since last shot)
    - 1: player did press space bar to shoot and game rules allowed it
- **player_tried**: indicator variable if player tried to shoot
    - 0: player did not press space bar to shoot
    - 1: player did press space bar to shoot   

## 4. [Bullets](#bullets)
## 5. [Enemies](#enemies)
## 6. [Events](#events)