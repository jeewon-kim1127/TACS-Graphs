import csv
import pandas as pd
from itertools import combinations


def check_pairs(a, b, filepath="path.csv", tolerance=1):
    with open(filepath, newline='') as csvfile:
        csvreader = csv.reader(csvfile)
        for row in csvreader:
            # 각 줄에서 숫자들을 float로 변환
            numbers = [float(num) for num in row]
            
            # a와 b에 대해 오차가 tolerance 이내인 숫자들이 있는지 확인
            a_within_tolerance = any(abs(num - a) <= tolerance for num in numbers)
            b_within_tolerance = any(abs(num - b) <= tolerance for num in numbers)
            
            if a_within_tolerance and b_within_tolerance:
                print(f"Found pair (a={a}, b={b}) with matching numbers in the row: {row}")
                return True  # 한 번 찾으면 바로 True 반환

    print(f"No matching pair (a={a}, b={b}) found.")
    return False  # 일치하는 쌍이 없다면 False 반환

candidate_file_path="/home/jeewonkim/sgraph_ws/workspaces/data/Indoor_3F/old/5.0/15.0_5.0_it3_time.csv"
# candidate_file_path="/home/jeewonkim/sgraph_ws/workspaces/data/Indoor_3F/traversable/15.0_it11_time.csv" #old_ablation #traversable
gt_file_path = "/home/jeewonkim/sgraph_ws/workspaces/s-graph/lidar_situational_graphs/src/s_graphs/eval/GT/Indoor10.csv"

count = 0
TF_count = 0
with open(candidate_file_path, newline='') as csvfile1:
    csvreader1 = csv.reader(csvfile1)
    next(csvreader1)
    for row in csvreader1:
        count+=1
        numbers = [float(num) for num in row]
        a= numbers[0]
        b= numbers[1]
        #indoor9
        # a= (numbers[0]+1657552579.76808-1654615953)#1.5)
        # b= (numbers[1]+1657552579.76808-1654615953) #1.5) 
        # if check_pairs(a,b,gt_file_path):
        #     TF_count += 1

        #indoor10_traversable
        diff = abs(a-b)
        if diff<0.2: #>10 and abs(diff-121.4)<0.2 :
            print(f"Found pair (a={a}, b={b})")
            TF_count += 1

print("precision ",TF_count/count)