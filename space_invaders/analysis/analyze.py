import pandas as pd
import json

# PATH = "../game_data/game_logs/"

def coop_check(data):
    df = pd.json_normalize(data, record_path = ['events'])
    coop = df[(df['killer'] == 'AI') & (df['killed'] == 'LEFT')]
    return len(coop)

def kill_summary(data):
    df = pd.json_normalize(data, record_path = ['events'])
    al = df[(df['killer'] == 'AI') & (df['killed'] == 'LEFT')]
    ar = df[(df['killer'] == 'AI') & (df['killed'] == 'RIGHT')]
    pl = df[(df['killer'] == 'PLAYER') & (df['killed'] == 'LEFT')]
    pr = df[(df['killer'] == 'PLAYER') & (df['killed'] == 'RIGHT')]
    p_deaths = df[(df['killed'] == 'PLAYER')]
    a_deaths = df[(df['killed'] == 'AI')]
    return ({'AI_left': len(al), 'AI_right': len(ar), 'P_left': len(pl), 'P_right': len(pr),
            'AI_deaths': len(a_deaths), 'P_deaths': len(p_deaths)})

def signals_summary(data):
    df = pd.json_normalize(data, record_path = ['frames'])
    down_count = df[df['signal_down'] == True]
    up_count = df[df['signal_up'] == True]
    tried_signal_down = df[df['tried_signal_down'] == True]
    tried_signal_up = df[df['tried_signal_up'] == True]
    return {'down_count': len(down_count), 'up_count': len(up_count),
            'tried_down_count': len(tried_signal_down), 'tried_up_count': len(tried_signal_up)}

def game_summary(filename):
    try:
        mode = 0
        with open(filename, 'r') as f:
            data = json.loads(f.read())
            if filename.find('_m1_') != -1:
                mode = 1
            elif filename.find('_m2_') != -1:
                mode = 2
            info = {'player_id': data['player_id'], 'date': data['date'], 'mode': mode}
            # if 'mode' in info.keys():
            #     info['mode'] = data['mode']

        return {**info, **kill_summary(data), **signals_summary(data)}
        # return {**info, **kill_summary(data)}
    except KeyError as k:
        print(f'Key error: {k}')
    except Exception as e:
        print(e)
        print(f'error: {filename}')


if __name__ == "__main__":
    s = game_summary('test.json')
    print(s)



# print(len(df_coop_late))
# # print(df_events.head())
