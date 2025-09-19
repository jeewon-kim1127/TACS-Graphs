import numpy as np
import yaml

def add_rectangle_to_pgm(png_array, resolution, rectangles, thickness):

    image_size = pgm_array.shape[0]
    for (x_min, x_max, y_min, y_max) in rectangles:
        #j_min = int((y_min / resolution) + (image_size / 2))-int(10/resolution)
        #j_max = int((y_max / resolution) + (image_size / 2))-int(10/resolution)
        j_max = image_size - (int((y_min / resolution) + (image_size / 2))-int(10/resolution))
        j_min = image_size - (int((y_max / resolution) + (image_size / 2))-int(10/resolution))
        i_min = int((-x_max / resolution) + (image_size / 2))-int(10/resolution)
        i_max = int((-x_min / resolution) + (image_size / 2))-int(10/resolution)
        
        # 두께를 고려한 검정 사각형 테두리 그리기
        pgm_array[i_min:i_min+thickness, j_min:j_max+1] = 0  # 상단
        pgm_array[i_max-thickness:i_max, j_min:j_max+1] = 0  # 하단
        pgm_array[i_min:i_max+1, j_min:j_min+thickness] = 0  # 좌측
        pgm_array[i_min:i_max+1, j_max-thickness:j_max] = 0  # 우측

    return png_array


def process_png_image(image_path, crop_coords, resolution, thickness=1):
    img = Image.open(image_path)
    png_array = np.array(img)

    modified_image_array = add_rectangle_to_pgm(png_array, resolution, crop_coords, thickness)
    
    result_img = Image.fromarray(modified_image_array)
    #result_img.save("output_with_rectangle.png")

    return result_img  # 처리된 이미지를 반환

if __name__ == '__main__':
    image_size = 2048
    resolution = 0.1
    image_path = '/home/jeewonkim/sgraph_ws/workspaces/tiers_ws/src/map_publisher/map_publisher/pgm/viz_3F.png"
    
    # 크롭 좌표 리스트 (실제 좌표)
    crop_coords = [
    (0.04, 2.57, 9.6, 11.2),
(3.44, 30.28, 10.2, 12),
(6.13, 35.12, 11.6, 12),
(5.98, 36.09, 9.4, 12),
(7.13, 33.28, 12, 15.6),
(8.29, 31.38, 12, 16),
(12.5, 27.74, 10, 14.6),
(9.33, 9.19, 10, 16.4),
(7.1, 1.03, 10.2, 16.4),
(7.19, -25.6, 5.8, 15.4),
(5.43, -33.27, 9.6, 14.6),
(1.21, -38.53, 15.8, 12.4),
(-1.26, -36.8, 15.8, 11),
(-0.54, -21.7, 6.2, 11),
(0.23, -7, 5.8, 11.8)
 ]
    
    processed_image = process_png_image(image_path, crop_coords, resolution, thickness=3)

    output_dir = '/home/jeewonkim/sgraph_ws/workspaces/tiers_ws/src/map_publisher/map_publisher/pgm'
    base_filename = '3F_room'
    rectangle_filename = f"{output_dir}/{base_filename}_roomseg_old.pgm"
    result_img.save(rectangle_filename)
    #save_pgm_image(pgm_with_rectangles, rectangle_filename)
    
    print(f"PGM with rectangles saved to: {rectangle_filename}")

