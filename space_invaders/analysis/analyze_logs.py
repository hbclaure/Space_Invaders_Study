from analyze import game_summary
import os
import pandas as pd

PATH = "../../data/pilot3/game_data/game_logs/"
save_name = "pilot"
# data = pd.read_csv('games.csv')

df = pd.DataFrame([game_summary(PATH + f) for f in os.listdir(PATH) if not f.startswith('.')])
df = df.sort_values(by=['player_id', 'date'])
n = 0
while (os.path.exists(f'{save_name}{n}.csv')):
    n += 1

save_path = f'{save_name}{n}.csv'
df.to_csv(save_path)
print(f'saved to: {save_path}')