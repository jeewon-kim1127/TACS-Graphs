import pandas as pd
from itertools import combinations

file_path = "/home/jeewonkim/sgraph_ws/workspaces/s-graph/lidar_situational_graphs/src/s_graphs/eval/GT/GT_indoor"
dataset_name="11"  
data = pd.read_csv(file_path+dataset_name+".csv", 
                    sep=r"\s+", names=["time", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"])

positions = data[["time", "x", "y", "z"]]

# 거리 계산 함수
def calculate_distance(row1, row2):
    return ((row1['x'] - row2['x'])**2 + (row1['y'] - row2['y'])**2) #+ (row1['z'] - row2['z'])**2)**0.5

# 거리 0.1 이하인 time 쌍 찾기
close_time_sets = []

print(len(positions),"계산시작")
idx1_tmp=500
for idx1 in positions.index:
    if abs(float(positions.loc[idx1]['time']) - float(positions.loc[idx1_tmp]['time'])) < 0.2 :
        continue
    existing = 0
    for existing_time in close_time_sets:
        if abs(float(positions.loc[idx1]['time']) - float(existing_time)) < 1.0:
            existing = 1
            continue
    if existing == 1 :
        continue
    close_time_sets = []

    for idx2 in positions.index:
        # if idx2<idx1:
        #     continue
        if abs(float(positions.loc[idx1]["time"])-float(positions.loc[idx2]["time"]))<0.3:
            continue
        # existing = 0
        # for existing_time in close_time_sets:
        #     if abs(float(positions.loc[idx2]['time']) - float(existing_time)) < 0.3 :
        #         existing = 1
        #         continue
        # if existing == 1 :
        #     continue

        dist = calculate_distance(positions.loc[idx1], positions.loc[idx2])
        if dist <= 0.2:
            close_time_sets.append(positions.loc[idx2]['time'])
        idx1_tmp = idx1
    if len(close_time_sets)>1 :
        print (positions.loc[idx1]['time'], close_time_sets)

# 결과 출력
# print("거리가 0.1 이하인 time 쌍:")
# for time_pair in close_time_sets:
#     print(time_pair)
