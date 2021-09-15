import tkinter as tk
from tkinter import filedialog
import os
import json
import sys
import time
from numpy import arctan2, pi, sqrt


def get_arguments():
    return sys.argv[1:]


def exe_time(init_time=0):
    if init_time:
        time_print = time.time() - init_time
        print("Planning finished in %f seconds" % time_print)
        return
    else:
        return time.time()


def load_json(path):
    directory = os.path.dirname(__file__)
    filename = os.path.join(directory, str(path))
    with open(filename, 'r', encoding="utf-8") as fin:
        return json.load(fin)


class PositionWindow(object):
    def __init__(self, master):
        top = self.top = tk.Toplevel(master)
        self.init_pos = [0, 0]
        self.goal_pos = [0, 0]
        self.l1 = tk.Label(top, text="Introducir las posiciones del robot [x,y]")
        self.l1.pack()
        self.l2 = tk.Label(top, text="x inicial y inicial x final y final")
        self.l2.pack()
        # self.l2.pack(side=tk.LEFT)
        self.e_x1 = tk.Entry(top)
        self.e_x1.pack()
        # self.e_x1.pack(side=tk.RIGHT)
        self.e_y1 = tk.Entry(top)
        self.e_y1.pack()
        # self.e_y1.pack(side=tk.RIGHT)
        self.e_x2 = tk.Entry(top)
        self.e_x2.pack()
        # self.e_x2.pack(side=tk.RIGHT)
        self.e_y2 = tk.Entry(top)
        self.e_y2.pack()
        # self.e_y2.pack(side=tk.RIGHT)
        self.b = tk.Button(top, text='Confirmar', command=self.cleanup)
        self.b.pack()

    def cleanup(self):
        self.init_pos = [int(self.e_y1.get()), int(self.e_x1.get())]
        self.goal_pos = [int(self.e_y2.get()), int(self.e_x2.get())]
        self.top.withdraw()
        self.top.quit()


class ModeWindow(object):
    def __init__(self, master):
        self.output = ""
        self.master = master
        self.master.geometry("500x200")
        self.master.title("Seleccionar el modo de entrada de las posiciones del robot")
        self.b = tk.Button(master, text="Indicar en el mapa", command=self.button1, width=60, height=6)
        self.b.pack(side=tk.TOP)
        self.b2 = tk.Button(master, text="Introducir manualmente", command=self.button2, width=60, height=6)
        self.b2.pack(side=tk.TOP)

    def button1(self):
        self.output = "map"
        self.close()

    def button2(self):
        self.output = "manual"
        self.close()

    def close(self):
        self.master.withdraw()
        self.master.quit()

    def kill(self):
        self.master.destroy()


def create_master():
    return tk.Tk()


def choose_map(master):
    master.withdraw()
    return filedialog.askopenfilename()


def load_params(filename = None):
    directory = os.path.dirname(__file__)
    if filename:
        filename = os.path.join(directory, filename)
    else:
        filename = os.path.join(directory, "param_config.json")
#     print(filename)
    if not os.path.exists(filename):
        raise ValueError("ValueError: Configuration file of parameters not found")

    with open(filename, 'r', encoding="utf-8") as fin:
        return json.load(fin)


def save_params(params):
    directory = os.path.dirname(__file__)
    directory = os.path.join(directory, "output")
    filename = os.path.join(directory, "param_config.json")
    if not os.path.exists(directory):
        os.makedirs(directory)
    with open(filename, 'w+', encoding="utf-8") as fin:
        json.dump(params, fin)
    return


def transform_path(path, mov_params, output_file=""):
    code_dict = {-90: "FORWARD",
                 90: "BACKWARD",
                 180: "LEFT",
                 0: "RIGHT",
                 -135: "FORWARD_LEFT",
                 -45: "FORWARD_RIGHT",
                 135: "BACKWARD_LEFT",
                 45: "BACKWARD_RIGHT"}

    def get_angle(cpos, npos):
        x_step = npos[0] - cpos[0]
        y_step = npos[1] - cpos[1]
        ang = int(arctan2(x_step, y_step)*180/pi)
        return ang

    directory = os.path.dirname(__file__)
    directory = os.path.join(directory, "output")
    filename = os.path.join(directory, output_file)
    if not os.path.exists(directory):
        os.makedirs(directory)

    steps_dict = {}
    for index in range(0, len(path)-2):
        angle = get_angle(path[index].position, path[index+1].position)
        step = code_dict[angle]
        letter = mov_params[step]
        steps_dict[index] = tuple([path[index].position[0], path[index].position[1], step, letter])
    steps_dict[len(path)-2] = tuple([path[len(path)-1].position[0], path[len(path)-1].position[1], "STOP", mov_params["STOP"]])

    with open(filename, 'w+', encoding="utf-8") as fin:
        json.dump(steps_dict, fin)
    return steps_dict


def create_route(steps, pixels_per_meter, output_file=""):
    def calculate_step(cpos, npos):
        x_step = npos[0] - cpos[0]
        y_step = npos[1] - cpos[1]
        return sqrt(x_step*x_step + y_step*y_step)

    directory = os.path.dirname(__file__)
    directory = os.path.join(directory, "output")
    filename = os.path.join(directory, output_file)
    if not os.path.exists(directory):
        os.makedirs(directory)

    step_size = 1/pixels_per_meter

    init_step = steps[0]
    prev_step = steps[1]
    step_counter = 0
    mov_counter = 0
    route = dict()
    route['Pixel_size'] = step_size
    route['Init_pos'] = [steps[0][1], steps[0][0]]
    route['Goal_pos'] = [steps[len(steps)-1][1], steps[len(steps)-1][0]]
    route_moves = dict()
    for step in steps.values():
        if step[3] != prev_step[3]:
            step_dist = calculate_step([init_step[0], init_step[1]], [prev_step[0], prev_step[1]])
            route_moves[mov_counter] = [step_dist*step_counter*step_size, step_counter, prev_step[2], prev_step[3]]
            step_counter = 1
            mov_counter += 1
        else:
            step_counter += 1
        init_step = prev_step
        prev_step = step

    route['Movements'] = route_moves

    with open(filename, 'w+', encoding="utf-8") as fin:
        json.dump(route, fin)

    return route
