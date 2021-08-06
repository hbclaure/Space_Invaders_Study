import itertools
from sklearn import svm, preprocessing, metrics
from sklearn.model_selection import train_test_split
from imblearn.over_sampling import RandomOverSampler, SMOTE, ADASYN


import pandas as pd
import numpy as np
import os
from collections import Counter
import random
#a = all_data[0]
# a.head()

# a.columns

def AU(n):
    # return AU column name
    return f'AU{str(n).zfill(2)}_r'

# define useful functions

def PROCESS(cols=[9, 14, 15, 17, 20, 23], range_val=1500, look_ahead_too=True, get_mean = False):
    count = 0
    # RANGE = 1000
    RANGE = range_val
    def closest_frame(df, f):
        f_near = f
        for x in range(50):
            while len(face_data.loc[face_data['frame'] == f_near, 'milis'].values) == 0:
                f_near -= 1
    #     print("number of frames retraced: " + str(f - f_near))
        return f_near

    def df_near(df, frame, ms_range=RANGE):
        """return rows where milis is within the search range"""
        if frame is None:
            return None
        ms = df.loc[df['frame'] == closest_frame(df,frame), 'milis'].values[0]
        if look_ahead_too:
            locs = df.loc[(df['milis'] >= ms - ms_range) & (df['milis'] <= ms + ms_range)]
        else:
            locs = df.loc[(df['milis'] >= ms - ms_range) & (df['milis'] <= ms)]
        return locs

    def max_df(d):
        if d is None:
            return None
        return pd.DataFrame(d.max()).transpose()

    def min_df(d):
        if d is None:
            return None
        return pd.DataFrame(d.min()).transpose()

    def mean_df(d):
        if d is None:
            return None
        return pd.DataFrame(d.mean()).transpose()

    def sd_df(d):
        if d is None:
            return None
        return pd.DataFrame(d.std()).transpose()



    # gather data

    # choose which action units to look at
    # AUs_to_look_at = [9, 14, 15, 17, 20, 23]
    AUs_to_look_at = cols

    def get_pid(filename):
        """ get pid from dirname """
        return filename.split('_')[0][1:]
    def get_mode(filename):
        """ get mode from dirname """
        return int(file.split('_')[2][1])

    up_data_max = []
    down_data_max = []
    up_data_min = []
    down_data_min = []
    all_data = []

    up_data_sd = []
    down_data_sd = []

    up_data_mean = []
    down_data_mean = []

    all_data_max = []
    all_data_min = []
    all_data_mean = []
    all_data_sd = []

    PATH = "/Users/yoony/Desktop/si_research/face_analysis/implicit_feedback_si_faceanalysis/processed"
    csvs = []
    for file in sorted(os.listdir(PATH)):
        if file[0] == '.':
            continue
        # print(file)
    #     csvs.append(pd.read_csv(os.path.join(PATH + "/" + file, 'webcam/aggregated_webcam.csv')))
        data = pd.read_csv(os.path.join(PATH + "/" + file, 'webcam/aggregated_webcam.csv'))
        face_data = data.drop(columns=list(data.columns.values[2:678]))
        face_data = face_data.drop(columns=list(face_data.columns.values[19:]))
        face_data = face_data[(['frame', 'milis'] + [AU(x) for x in AUs_to_look_at])]
        
        signals = pd.read_csv('../my_buckets.csv')[['player_id', 'mode', 'signal_type', 'frame_number', 'code']]
        s1 = signals.loc[(signals['player_id'] == get_pid(file)) & (signals['mode'] == get_mode(file))]
        s_up = s1.loc[s1['signal_type'] == 'up'].frame_number.values
        s_down = s1.loc[s1['signal_type'] == 'down'].frame_number.values
        
        for x in s_up:
            up_locs = df_near(face_data, x)
            up_data_max.append(max_df(up_locs))
            up_data_min.append(min_df(up_locs))
            up_data_sd.append(sd_df(up_locs))
            up_data_mean.append(mean_df(up_locs))
            
            # all_data_max.append(max_df(up_locs))
            # all_data_min.append(min_df(up_locs))
            # all_data_sd.append(sd_df(up_locs))
            # all_data_mean.append(mean_df(up_locs))
            
        for x in s_down:
            down_locs = df_near(face_data, x)
            down_data_max.append(max_df(down_locs))
            down_data_min.append(min_df(down_locs))
            down_data_sd.append(sd_df(down_locs))
            down_data_mean.append(mean_df(down_locs))
            
            # all_data_max.append(max_df(down_locs))
            # all_data_min.append(min_df(down_locs))
            # all_data_sd.append(sd_df(down_locs))
            # all_data_mean.append(mean_df(down_locs))
            
        all_data.append(face_data)
        
        



    def rename_columns(df, tag):
        a = df.copy()
        a.columns = list(map(lambda x: x + tag, a.columns))
        return a

    def merge_dfs(dfs):
        a = dfs[0]
        for d in dfs[1:]:
            a = a.join(d, how='outer')
        return a
            
    u1 = rename_columns(pd.concat(up_data_max).drop(columns=['frame', 'milis']), '_max').reset_index(drop=True)
    u2 = rename_columns(pd.concat(up_data_min).drop(columns=['frame', 'milis']), '_min').reset_index(drop=True)
    u3 = rename_columns(pd.concat(up_data_mean).drop(columns=['frame', 'milis']), '_mean').reset_index(drop=True)
    u4 = rename_columns(pd.concat(up_data_sd).drop(columns=['frame', 'milis']), '_sd').reset_index(drop=True)


    # ups = merge_dfs([u1, u2, u3, u4])
    ups = merge_dfs([u1,u2,u3,u4])
    ups['type'] = 'up'


    d1 = rename_columns(pd.concat(down_data_max).drop(columns=['frame', 'milis']), '_max').reset_index(drop=True)
    d2 = rename_columns(pd.concat(down_data_min).drop(columns=['frame', 'milis']), '_min').reset_index(drop=True)
    d3 = rename_columns(pd.concat(down_data_mean).drop(columns=['frame', 'milis']), '_mean').reset_index(drop=True)
    d4 = rename_columns(pd.concat(down_data_sd).drop(columns=['frame', 'milis']), '_sd').reset_index(drop=True)



    # downs = merge_dfs([d1, d2, d3, d4])
    downs = merge_dfs([d1,d2,d3,d4])
    downs['type'] = 'down'


    # In[13]:


    means = pd.DataFrame(pd.concat(up_data_mean).mean(), columns=['up_mean'])
    means['down_mean'] = pd.concat(down_data_mean).mean()
    means['mean_diff'] = means['up_mean'] - means['down_mean']
    means['up_sd'] = pd.concat(up_data_max).std()
    means['down_sd'] = pd.concat(down_data_max).std()
    means['sd_diff'] = means['up_sd'] - means['down_sd']
    means = means.drop(index=['frame', 'milis'])
    # means['total'] = pd.concat(all_data).mean
    # means.to_csv('face/au_data.csv')
    means


    # In[14]:


    # check number of signals
    len(up_data_max) + len(down_data_max)


    # In[15]:


    clf = svm.SVC(kernel='linear', class_weight='balanced')


    # In[16]:


    #clean up and finalize the data
    signal_data = pd.concat([ups, downs]).reset_index(drop=True)
    signal_data
    signal_types = signal_data['type'].values
    signal_data = signal_data.drop(columns=['type'])


    # In[17]:


    X = signal_data
    y = signal_types


    # normalize first
    def result(over_type):
        seed = 109 #random seed
        seed = random.randint(0,200)
        
        over = { 'random': RandomOverSampler(random_state=seed),
                'smote': SMOTE(random_state=seed),
                'adasyn': ADASYN(random_state=seed)}
        
    #     ros = RandomOverSampler(random_state=seed)
    #     smote = SMOTE(random_state=seed)
    #     adasyn = ADASYN(random_state=seed)
        
        

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=seed)

        # normalize inputs, use fit_transform for train and transform for test
        normalizer = preprocessing.Normalizer()
        norm_X_train = normalizer.fit_transform(X_train)
        norm_X_test = normalizer.transform(X_test)

        X_train_resampled, y_train_resampled = over[over_type].fit_resample(norm_X_train, y_train)
        # X_train_resampled, y_train_resampled = under.fit_resample(X_train_resampled, y_train_resampled)

        # X_train_resampled, y_train_resampled = smote.fit_resample(norm_X_train, y_train)
        # X_train_resampled, y_train_resampled = X_train, y_train

        # fit data
        clf.fit(norm_X_train, y_train)
        # predict
        y_pred = clf.predict(norm_X_test)
        return metrics.f1_score(y_test, y_pred, average=None)


    # for i in range(100):
    #     result()[0]
    if get_mean:
        r = [result('random')[0] for i in range(50)]
        s = [result('smote')[0] for i in range(50)]
        a = [result('adasyn')[0] for i in range(50)]
        return (f'random:  {np.mean(r)}, smote:  {np.mean(s)}, adasyn: {np.mean(a)}')
    else:
        r = result('random')
        s = result('smote')
        a = result('adasyn')
        return (f'random:  {r}, smote:  {s}, adasyn: {a}')



aus = [9, 14, 15, 17, 20, 23, 45]
from itertools import chain, combinations, product

combs = ( list(itertools.combinations(aus, 4)) + 
          list(itertools.combinations(aus, 5)) +
          list(itertools.combinations(aus, 6)) +
          list(itertools.combinations(aus, 7)) + 
          list(itertools.combinations(aus, 8)))
for x in combs:
    if len(x) >= 4:
        with open('results.txt', 'a') as f:
            f.write(str(x) + PROCESS(cols=x, get_mean=True) + '\n')

# print(PROCESS())