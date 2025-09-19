import numpy as np
from PIL import Image, ImageDraw

def save_png_image(array, filename):
    """
    PNG 이미지 저장 함수
    """
    img = Image.fromarray(array)
    img.save(filename)

def get_min_max_coordinates_from_pcd(image_size, pcd_file_path, altitude, resolution):
    array_2d = [[255 for _ in range(image_size)] for _ in range(image_size)]
    with open(pcd_file_path, 'r') as file:
        lines = file.readlines()
    data_lines = lines[:]
    x_values = []
    y_values = []
    z_values = []
    mid = int(image_size / 2)
    array_2d[mid][mid] = 0
    for line in data_lines:
        values = [float(v) for v in line.split(',')]
        x_values.append(values[0])
        y_values.append(values[1])
        z_values.append(values[2])
        j = values[0] / resolution + image_size / 2
        i = -values[1] / resolution + image_size / 2
        if (abs(values[2] - altitude)) < 0.2:
            if 0 <= i < image_size and 0 <= j < image_size:
                array_2d[int(i)][int(j)] = 150

    min_x, max_x = min(x_values), max(x_values)
    min_y, max_y = min(y_values), max(y_values)
    min_z, max_z = min(z_values), max(z_values)

    return min_x, max_x, min_y, max_y, min_z, max_z, array_2d

def add_rectangle_to_png(png_array, resolution, rectangles, thickness):
    """
    PNG 이미지에 빨간색 사각형 추가 함수
    """
    image_size = png_array.shape[0]
    
    # PNG 이미지를 열어서 드로잉 가능한 상태로 변환
    img = Image.fromarray(png_array.astype(np.uint8))  # 데이터 타입을 uint8로 변환
    draw = ImageDraw.Draw(img)

    # 빨간색 RGB 값 (정수값으로 명시)
    red = (255, 0, 0)

    for (x_min, x_max, y_min, y_max) in rectangles:
        #j_min = int((y_min / resolution) + (image_size / 2)) - int(13 / resolution)
        #j_max = int((y_max / resolution) + (image_size / 2)) - int(13 / resolution)
        #i_min = int((-x_max / resolution) + (image_size / 2)) - int(50 / resolution)
        #i_max = int((-x_min / resolution) + (image_size / 2)) - int(50 / resolution)
        
        i_min = int((y_min / resolution) + (image_size / 2)) 
        i_max = int((y_max / resolution) + (image_size / 2))
        j_min = int((-x_max / resolution) + (image_size / 2)) 
        j_max = int((-x_min / resolution) + (image_size / 2)) 
        
        if j_min > j_max:
            j_min, j_max = j_max, j_min
        if i_min > i_max:
            i_min, i_max = i_max, i_min
        
        # 빨간색 사각형 테두리 그리기
        for i in range(thickness):
            # 상단, 하단, 좌측, 우측 테두리 그리기
            draw.line([(j_min, i_min + i), (j_max, i_min + i)], fill='red')
            draw.line([(j_min, i_max - i), (j_max, i_max - i)], fill='red')
            draw.line([(j_min + i, i_min), (j_min + i, i_max)], fill='red')
            draw.line([(j_max - i, i_min), (j_max - i, i_max)], fill='red')

    return np.array(img)


if __name__ == '__main__':
    image_size = 2048
    resolution = 0.1
    altitude = 0.5
    pcd_file_path = '/home/jeewonkim/sgraph_ws/workspaces/tiers_ws/src/map_publisher/map_publisher/pcd/3F.asc'
    output_dir = '/home/jeewonkim/sgraph_ws/workspaces/tiers_ws/src/map_publisher/map_publisher/pgm'
    base_filename = '3F'
    
    min_x, max_x, min_y, max_y, min_z, max_z, png_array = get_min_max_coordinates_from_pcd(image_size, pcd_file_path, altitude, resolution)
    png_array = np.array(png_array)

    # 크롭 좌표 리스트 (실제 좌표)
    crop_coords = [
    (-2.88,3.32,3.29,10.88),
    (1.01,11.88,-1.18,5.49),
    (-7.93,4.91,-42.86,-27.12),
    (-5.03,6.26,-10.9,12.75),
    (3.87,12.7,34.04,40.95),
    (8.05,14.29,13.68,42.18),
    (-2.54,8.74,-43.92,-20.13),
    (-5.02,6.74,-47.06,-36.08)

      #  (0.04, 2.57, 9.6, 11.2), (3.44, 30.28, 10.2, 12), (6.13, 35.12, 11.6, 12),
      #  (5.98, 36.09, 9.4, 12), (7.13, 33.28, 12, 15.6), (8.29, 31.38, 12, 16),
      #  (12.5, 27.74, 10, 14.6), (9.33, 9.19, 10, 16.4), (7.1, 1.03, 10.2, 16.4),
      #  (7.19, -25.6, 5.8, 15.4), (5.43, -33.27, 9.6, 14.6), (1.21, -38.53, 15.8, 12.4),
      #  (-1.26, -36.8, 15.8, 11), (-0.54, -21.7, 6.2, 11), (0.23, -7, 5.8, 11.8)
    ]

    # 빨간색 사각형 추가
    png_with_rectangles = add_rectangle_to_png(png_array, resolution, crop_coords, thickness=3)

    # 수정된 PNG 저장
    rectangle_filename = f"{output_dir}/{base_filename}_traversable_roomseg.png"
    save_png_image(png_with_rectangles, rectangle_filename)
    print(f"PNG with rectangles saved to: {rectangle_filename}")

