import numpy as np
from PIL import Image  #pip install Pillow
import yaml  # pip install pyyaml
from scipy.spatial import KDTree
from scipy.ndimage import label

def save_pgm_image(array, filename):
    normalized_array = ((array - np.min(array)) / (np.max(array) - np.min(array))) * 255
    normalized_array = normalized_array.astype(np.uint8)
    # PGM 이미지 파일 헤더 작성
    height, width = normalized_array.shape
    header = f"P2\n{width} {height}\n255\n"
    # PGM 이미지 파일 작성
    with open(filename, 'w') as file:
        file.write(header)
        np.savetxt(file, normalized_array, fmt='%d', delimiter=' ')

def remove_isolated_pixels(array_2d, min_size=5):
    array_list = np.array(array_2d)
    labeled_array, num_features = label(array_list == 0)  # 0 (검은 점) 기준 연결성 검사


    for label_idx in range(1, num_features + 1):
        # 각 연결 영역의 픽셀 수 계산
        print(np.sum(labeled_array == label_idx))
        if np.sum(labeled_array == label_idx) < min_size:
            array_list[labeled_array == label_idx] = 255  # 고립된 그룹 제거
    return array_list.tolist()

def get_min_max_coordinates_from_pcd(image_size, pcd_file_path,altitude, resolution):
    array_2d = [[255 for _ in range(image_size)] for _ in range(image_size)]
    with open(pcd_file_path, 'r') as file:
        lines = file.readlines()
    # 데이터 시작 라인 찾기
    data_start = lines.index("DATA ascii\n") + 1 if "DATA ascii\n" in lines else None
    if data_start is None:
        raise ValueError("Invalid PCD file format. DATA ascii not found.")
    # 데이터 부분만 추출
    data_lines = lines[data_start:]
    # x, y, z 좌표 추출 및 최소값 및 최대값 계산
    x_values = []
    y_values = []
    z_values = []

    for line in data_lines:
        values = [float(v) for v in line.split()]
        x_values.append(values[0])
        y_values.append(values[1])
        z_values.append(values[2])
        j= values[0]/resolution + image_size/2
        i= -values[1]/resolution + image_size/2
        if((values[2] - altitude)<0.2):
            if 0 <= i < image_size and 0 <= j < image_size:
                array_2d[int(i)][int(j)]=0

    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    min_z, max_z = min(z_values), max(z_values)

    return min_x, max_x, min_y, max_y, min_z, max_z,array_2d


if __name__ == '__main__':
    image_size = 2048
    resolution = 0.1
    altitude = 0 #3F #1.0 #global_cloud_lab  #2.0 #1F   # 2.0 # 6F

    pcd_file_path = '/home/jeewonkim/sgraph_ws/workspaces/tiers_ws/src/map_publisher/map_publisher/pcd/3F.asc' #global_cloud_1th.pcd'  #<== input pcd file name
    output_file_name='/home/jeewonkim/sgraph_ws/workspaces/tiers_ws/src/map_publisher/map_publisher/pgm/3F'           #<== output file name

    pgm_file_name=output_file_name+'.pgm'
    yaml_file_name = output_file_name+'.yaml'
    # 최소 최대값 출력
    min_x,max_x, min_y,max_y, min_z,max_z,array_2d = get_min_max_coordinates_from_pcd(image_size, pcd_file_path,altitude,resolution)
    x_range= max_x - min_x
    y_range= max_y - min_y
    z_range= max_z - min_z
    print(min_x,max_x, min_y,max_y, min_z,max_z)

    # 고립된 픽셀 제거
    #array_filtered = remove_isolated_pixels(array_2d, 2)

    # PGM 저장
    print(f"Image saved to {pgm_file_name}")
    #save_pgm_image(array_filtered, pgm_file_name)
    save_pgm_image(array_2d, pgm_file_name)

    print(f"YAML file '{yaml_file_name}' has been created.")
    data = {
        'mode': 'trinary',
        'occupied_thresh': 0.95,
        'free_thresh': 0.90,
        'negate': 0,
        'origin': [-image_size*resolution/2, -image_size*resolution/2, 0.0],
        'resolution': resolution,
        'image': pgm_file_name
    }
    with open(yaml_file_name, 'w') as file:
        yaml.dump(data, file, default_flow_style=False)
