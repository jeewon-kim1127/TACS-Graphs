import csv
import math

# 누적 이동거리 계산 함수
def calculate_accumulated_distance(csv_file, column_index):
    positions = []  # (x, y) 좌표를 저장할 리스트

    # CSV 파일 읽기
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            if len(row) > column_index:
                # 특정 셀 값을 스페이스바 기준으로 분리
                cell_values = row[column_index].split()
                if len(cell_values) >= 3:
                    try:
                        x = float(cell_values[1])  # 두 번째 값
                        y = float(cell_values[2])  # 세 번째 값
                        positions.append((x, y))
                    except ValueError:
                        print(f"Invalid float conversion in row: {row}")

    # 누적 이동거리 계산
    accumulated_distance = 0.0
    for i in range(1, len(positions)):
        x1, y1 = positions[i - 1]
        x2, y2 = positions[i]
        # 두 점 사이 거리 계산
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        accumulated_distance += distance

    return accumulated_distance

# CSV 파일과 열 인덱스 설정
file_path = "/home/jeewonkim/sgraph_ws/workspaces/s-graph/lidar_situational_graphs/src/s_graphs/eval/GT/GT_indoor"
dataset_name="09"  
csv_file_path = file_path+dataset_name+".csv"
# data = pd.read_csv(file_path+dataset_name+".csv",
#                     sep=r"\s+", names=["time", "x", "y", "z", "q_x", "q_y", "q_z", "q_w"])

column_index = 0  # 스플릿할 셀이 있는 열 인덱스

# 누적 이동거리 계산 및 출력
accumulated_distance = calculate_accumulated_distance(csv_file_path, column_index)
print(f"Accumulated Distance: {accumulated_distance}")
