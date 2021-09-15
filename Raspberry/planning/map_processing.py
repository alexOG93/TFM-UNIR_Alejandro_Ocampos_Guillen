import numpy as np
import matplotlib.pyplot as plt
from matplotlib.image import imread
import tifffile
import cv2
import os


def get_point(map_bin: np.array):
    map_bin = map_bin*255
    cv2.imwrite("temp_img.jpg", map_bin)
    im = cv2.imread("temp_img.jpg")
    cv2.namedWindow("Image", 2)
    cv2.resizeWindow("Image", im.shape[0], im.shape[1])
    r = cv2.selectROI("Image", im, False, False)
    cv2.destroyWindow("Image")
    os.remove("temp_img.jpg")
    return [int(r[1]+r[3]/2), int(r[0]+r[2]/2)]


def resize_map(image_binary, params):
    resize_factors = [10, 5, 2, 1]
    resize_factor = 1
    x_len, y_len = image_binary.shape
    for factor in resize_factors:
        if ((params["PIXELS_PER_METER"]/factor) * params["CAR_SIZE"]) % 1 == 0:
            resize_factor = factor
            break

    new_x_len = int(x_len/resize_factor)
    new_y_len = int(y_len/resize_factor)
    new_binary = np.zeros([new_x_len, new_y_len],  dtype='uint8')

    kernel = np.ones([resize_factor, resize_factor],  dtype='uint8')
    for y_index in range(0, new_y_len):
        yb_index = resize_factor*y_index
        for x_index in range(0, new_x_len):
            xb_index = resize_factor * x_index
            mult_matrix = image_binary[xb_index:(xb_index+resize_factor), yb_index:(yb_index+resize_factor)]
            multiplication = np.matmul(mult_matrix, kernel)
            if multiplication.sum().sum() == resize_factor*resize_factor*resize_factor:
                # raise SyntaxError("Falta definir cuando debe ponerse un 1 en el mapa, dependerá del tamaño ")
                new_binary[x_index][y_index] = 1
    params["PIXELS_PER_METER"] /= resize_factor
    return new_binary


def get_bin_map(filename):
    def bin_map(pixel):
        pixel = int(round(pixel, 0))
        if pixel not in walls:
            pixel = 1
        else:
            pixel = 0
        return pixel

    if filename[-4:] == ".tif":
        img = tifffile.imread(filename)
        img = img[130:, 130:, :]
    elif filename[-4:] == ".png":
        img = imread(filename)
    elif filename == "":
        raise ValueError("ValueError: No map was selected")
    else:
        raise ValueError(f"ValueError: Unrecognized map file type: {filename}")
    if len(img.shape) > 2:
        if img.shape[2] > 1:
            grey = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2])
        else:
            grey = img
    else:
        grey = img

    im_values = np.unique(grey)
    if len(im_values) > 2:
        img_shape = grey.shape
        flat_grey = np.ndarray.flatten(grey)
        walls = np.array([0, 37, 226, 237])
        binary = np.array(list(map(bin_map, flat_grey)))
        binary = binary.reshape((img_shape[0], img_shape[1]))
    else:
        binary = grey.astype(int)
    return binary


def print_map(map_in):
    plt.imshow(map_in, cmap='gray')
    # plt.show(block=False)
    plt.show()
    return


def save_map(map_in, output_file="map.png"):
    if os.name == "nt":
        if not os.path.exists("output"):
            os.makedirs("output")
        output_file = os.path.join("output", output_file)
        cv2.imwrite(output_file, map_in)
        return
    else:
        directory = os.path.dirname(__file__)
        directory = os.path.join(directory, "output")
        if not os.path.exists(directory):
            os.makedirs(directory)
        output_file = os.path.join(directory, output_file)
        cv2.imwrite(output_file, map_in)
        return


def update_map(map_bin, map_proc, input_domain, car_size, pixels_per_meter, meters_margin):
    def add_margin(x_init, x_obs, obs_marg):
        proximity = x_obs - x_init
        if abs(proximity) < obs_marg and proximity != 0:
            if proximity > 0:
                new_x = x_init + obs_marg
            elif proximity < 0:
                new_x = x_init - obs_marg
            else:
                raise ValueError("Input configuration places the robot on the position of an obstacle")

        else:
            new_x = x_obs
        # ~ print(f"new_x is {new_x}")
        # ~ print(f"margin is {obs_marg}")
        return int(round(new_x, 0))
    init_pos = input_domain["Init_pos"][::-1]
    obstacle_pos = input_domain["Obstacle_pos"][::-1]
    margin = int(round((meters_margin + car_size/2)*pixels_per_meter, 0))
    obstacle_pos[1] = add_margin(init_pos[1], obstacle_pos[1], margin)
    obstacle_pos[0] = add_margin(init_pos[0], obstacle_pos[0], margin)
    # Add obstacle to map
    map_bin[obstacle_pos[0]][obstacle_pos[1]] = 0
    for incrX in range(0, margin):
        for incrY in range(0, margin):
            map_proc[obstacle_pos[0] - incrX][obstacle_pos[1] - incrY] = 0
            map_proc[obstacle_pos[0] - incrX][obstacle_pos[1] + incrY] = 0
            map_proc[obstacle_pos[0] + incrX][obstacle_pos[1] - incrY] = 0
            map_proc[obstacle_pos[0] + incrX][obstacle_pos[1] + incrY] = 0
    return map_bin, map_proc


def map_safety_margin(map_in, car_size, pixels_per_meter, meters_margin):
    pixels_margin = meters_margin*pixels_per_meter
    kernel_size = int(np.ceil(car_size*pixels_per_meter) + pixels_margin*2)
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    return cv2.erode(map_in.astype('uint8'), kernel, iterations=1)


def add_path_to_map(binary_map, path, car_size, pixels_per_meter):
    map_with_path = np.zeros((binary_map.shape[0], binary_map.shape[1]), dtype='uint8')
    car_pixels = int(car_size * pixels_per_meter)
    if car_pixels % 2:
        kernel_size = int(np.ceil(car_pixels))
    else:
        kernel_size = int(np.ceil(car_pixels)) + 1
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    for state in path:
        map_with_path[state.position[0]][state.position[1]] = 150
    map_with_path = cv2.dilate(map_with_path, kernel, iterations=1)
    for state in path:
        map_with_path[state.position[0]][state.position[1]] = 200
    map_with_path += binary_map.astype('uint8')*255

    return map_with_path
