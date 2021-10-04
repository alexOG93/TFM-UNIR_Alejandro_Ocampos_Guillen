#!/usr/bin/env python

import rospy, serial
from std_msgs.msg import String
from sensor_msgs.msg import Range
import sys, tty, termios, signal
import json
import Tkinter as tk
import tkFileDialog as filedialog
import os
import io

SPEED = 13 # cm/s 14.5
SPEED_DIAG = 8.5 # cm/s

# key reserved as default, will stop the robot
STOP_KEY = "p"
DEFAULT_KEY = "s"

COM_RATE = 2
obs_det_msg = 3
user_interrupt = False

def get_obstacle_alarm(message):
    global obs_det_msg
    obs_det_msg = message.data


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
    return ch.lower()


def sigint_handler(signal, frame):
    global user_interrupt
    user_interrupt = True
    print("KeyboardInterrupt happened")
    return


def no_obs_detected(obs_flags, move):
    # Sensors order in flag list: 0: Front, 1: Right, 2: Left, 3: Back
    sensors_to_check = {"w": [0], "q": [0,1], "e": [0,2], "a": [1], "d": [2], "z": [1,3], "x": [3] , "c": [2,3]}
    try:
        sensors = sensors_to_check[move]
    except:
        return True
    for sensor_index in sensors:
        if obs_flags[sensor_index] == "o": # "o" means cell is occupied, there is an obstacle
            return False
    return True
    
    
def load_path(filename = None):
    if not filename:
        print("Please, select a route file")
        root = tk.Tk()
        root.withdraw()
        filename = filedialog.askopenfilename()
        root.destroy()
        del root
    
    if filename[-5:] == ".json":
        with open(filename, 'r') as fin:
            path = json.load(fin)
        for needed_key in ["Pixel_size", "Init_pos", "Goal_pos", "Movements"]:
            if needed_key not in path:
                raise ValueError("Route file with missing keys")
        
    elif filename == "":
        raise ValueError("No file was selected")
    else:
        raise ValueError("Unrecognized file type: %s" % filename)
        
    return path
    

def update_pos(route, path, new_mov, mov_time, goal_time):
    def add_steps(position, mov_type, steps):
        x_movs = ["LEFT", "FORWARD_LEFT", "BACKWARD_LEFT", "RIGHT", "FORWARD_RIGHT", "BACKWARD_RIGHT"]
        y_movs = ["FORWARD", "FORWARD_RIGHT", "FORWARD_LEFT", "BACKWARD", "BACKWARD_RIGHT", "BACKWARD_LEFT"]
        if mov_type in x_movs[:3]:
            position[0] -= steps
        elif mov_type in x_movs[3:]:
            position[0] += steps
        if mov_type in y_movs[:3]:
            position[1] -= steps
        elif mov_type in y_movs[3:]:
            position[1] += steps
        position[0] = int(position[0])
        position[1] = int(position[1])
        return position
    
    stop_position = route["Init_pos"]
    for i in range(0,new_mov):
        mov_type = path[str(i)][2]
        num_steps = path[str(i)][1]
        stop_position = add_steps(stop_position, mov_type, num_steps)            
    mov_type = path[str(new_mov)][2]
    mov_time2 = mov_time + rospy.Duration.from_sec(0.5)
    num_steps = round((path[str(new_mov)][1]*((mov_time2)/goal_time)),0)
    stop_position = add_steps(stop_position, mov_type, num_steps)

    return stop_position


def get_obs_pos(position, mov_type, obstacle_distance = 1):
    x_movs = ["LEFT", "FORWARD_LEFT", "BACKWARD_LEFT", "RIGHT", "FORWARD_RIGHT", "BACKWARD_RIGHT"]
    y_movs = ["FORWARD", "FORWARD_RIGHT", "FORWARD_LEFT", "BACKWARD", "BACKWARD_RIGHT", "BACKWARD_LEFT"]
    steps = obstacle_distance
    obstacle_pos = position[:]
    if mov_type in x_movs[:3]:
        obstacle_pos[0] -= steps
    elif mov_type in x_movs[3:]:
        obstacle_pos[0] += steps
    if mov_type in y_movs[:3]:
        obstacle_pos[1] -= steps
    elif mov_type in y_movs[3:]:
        obstacle_pos[1] += steps
    obstacle_pos[0] = int(obstacle_pos[0])
    obstacle_pos[1] = int(obstacle_pos[1])
    # ~ print("Obs_pos should be:")
    # ~ print(obstacle_pos[0])
    # ~ print(obstacle_pos[1])
    return obstacle_pos
    
    
def write_config(stop_position, goal_pos, obstacle_pos = None):
    replan_config = dict()
    replan_config["Init_pos"] = stop_position
    replan_config["Goal_pos"] = goal_pos
    if obstacle_pos:
        replan_config["Obstacle_pos"] = obstacle_pos
    
    directory = os.path.dirname(__file__)
#     print(directory)
    filename = "exists"
#     for root, dirs, files in os.walk(directory):
#         for filename in files:
#             if filename[-4:] in [".png", ".tif"]:
#                 map_file = filename
#                 break
#         if filename:
#             break
    output_path = os.path.join("planning", "output")
    map_file = os.path.join(output_path, "map_binary.png")
    params_file = os.path.join(output_path, "param_config.json")
    if not filename:
        print("Please, select image file containing the map")
        master = tk.Tk()
        master.withdraw()
        map_file = filedialog.askopenfilename()
        master.destroy()
        del master
    map_file = os.path.join(directory, map_file)
    params_file = os.path.join(directory, params_file)
    replan_config["Map_file"] = map_file
    replan_config["Params_file"] = params_file
    
    directory = os.path.dirname(__file__)
    filename = os.path.join(directory, "replan_config.json")
    
    with open(filename, 'w') as fin:    
        json.dump(replan_config, fin)
    
    return filename
    

def remote_control_main():
    # Suscribers to ultrasonics sensor measurements
    obs_det_subs = rospy.Subscriber("obstacle_avoidance", String, get_obstacle_alarm)
    # ~ key_subs = rospy.Subscriber("key_captured", keyboard_msg, get_key)
    
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    ser.flush()
    
    global user_interrupt
    rate = rospy.Rate(COM_RATE)
    ch = STOP_KEY
    key_to_publish = STOP_KEY    
    new_mov = -1
    prev_mov = -1
    init_time = rospy.Duration.from_sec(0)
    route_file = ""
    
    # Flags needed
    GBroute_mode = 0
    obs_det_flag = False
    states = {
        "manual_driving": 0,
        "load_route": 1,
        "exe_route": 2,
        "replan_route": 3,
        "load_GB_route": 4,
        "plan_route": 5
        }
    robot_state = "manual_driving"
    print("remote_serialctrl_node node is running succesfully, and keys pressed in this terminal will be captured.\n" +
            "Press \"o\" to exit during manual driving mode, and Ctrl+C during route mode")   
        
    while not rospy.is_shutdown():
#         print(robot_state)
#         print(GBroute_mode)
        ch = STOP_KEY
        signal.signal(signal.SIGINT, sigint_handler)
        if user_interrupt:
            print("Evaluating")
            pos = update_pos(route, path, new_mov, mov_time, goal_time)
            print("Robot current position is (%i, %i)" % (pos[1], pos[0]))
            sys.exit(0)
        if states[robot_state] == states["manual_driving"]:
            ch = getch()        
            # keys reserved to exit the node execution
            if ch == "o":
                ser.write((STOP_KEY+"\n").encode('utf-8'))
                break
            # ~ print(obs_det_msg)
            elif ch == "p":
                # Planning mode
                robot_state = "plan_route"
            elif ch == "r":
                route_file = None
                print("Initiating route mode...")
                robot_state = "load_route"
            elif ch == "v":
                GBroute_mode = 2
                route_file = None
                print("Initiating roundtrip route mode...")
                robot_state = "load_route"
                
            
        elif states[robot_state] == states["load_route"]:
            if route_file:
                route = load_path(filename=route_file)
            else:
                print("Route mode on. Please, load path file")
                route = None
                try:
                    route = load_path()
                except Exception as excep:
                    print(repr(excep))
                
            if not route:
                print("Route file loading failed. Please check the selected file")
                print("Returning to manual driving mode")
                robot_state = "manual_driving"
            else:          
                print("Path correctly loaded. Robot will execute it now")
                path = route["Movements"]
                path_len = len(path)
                new_mov = 0
                prev_mov = -1
                if GBroute_mode and not obs_det_flag:
                    robot_state = "load_GB_route"
                else:
                    robot_state = "exe_route"
                    obs_det_flag = False
            
        elif states[robot_state] == states["exe_route"]:
            diag_movs = ["q", "e", "z", "c"]
            if new_mov != prev_mov:
#                 print("New movement loaded %i"% new_mov)
                path_key = path[str(new_mov)][3]
                if path_key in diag_movs:
                    speed_mov = SPEED_DIAG
                else:
                    speed_mov = SPEED
                goal_time = rospy.Duration.from_sec(100*path[str(new_mov)][0]/speed_mov) # cm / (cm/s)
                init_time = rospy.get_rostime()
                prev_mov = new_mov
                ch = path_key
            else:
                ch = path_key
                mov_time = rospy.get_rostime() - init_time
                if mov_time >= goal_time:
                    new_mov += 1                    
                    if new_mov >= path_len:
                        if not GBroute_mode:
                            print("Path completed. Route mode off")
                            print("Returning to manual driving mode")
                            route_file = None
                            ch = STOP_KEY
                            robot_state = "manual_driving"
                            new_mov = -1
                            prev_mov = -1
                            init_time = rospy.Duration.from_sec(0)
                        else:
                            print("First path completed. Now, robot will plan the path to go back to its initial position")
                            robot_state = "load_GB_route"
        elif states[robot_state] == states["replan_route"]:
            # json file for replanning
            # calculate position
#             stop_position = route["Init_pos"]
#             for i in range(0,new_mov):
#                 mov_type = path[str(i)][2]
#                 num_steps = path[str(i)][1]
#                 stop_position = update_pos(stop_position, mov_type, num_steps)            
#             mov_type = path[str(new_mov)][2]
#             num_steps = round((path[str(new_mov)][1]*(mov_time/goal_time)),0)
            stop_position = update_pos(route, path, new_mov, mov_time, goal_time)
#             print(stop_position[0])
#             print(stop_position[1])
            # Calculate obstacle position
            mov_type = path[str(new_mov)][2]
            obstacle_position = get_obs_pos(stop_position, mov_type)
            
            # Write reroute file
            config_file = write_config(stop_position, route["Goal_pos"], obstacle_position)
            
            directory = os.path.dirname(__file__)
            directory = os.path.join(directory, "planning")
            command = "python3 " + directory + "/main_planning.py" + " " + config_file
            # Execute planning
            print("Executing replanning of best path to goal...")
            try:
                os.system(command)
                if not os.path.exists(os.path.join(directory, "output", "route.json")):
                    robot_state = "manual_driving"
                    print("Replaning process failed")
                    print("Current robot position is [%i, %i]" % (stop_position[1], stop_position[0]))
                    print("Returning to manual driving mode")
                else:
                    route_file = directory + "/output/route.json" # This file should be created by previous line
                    robot_state = "load_route"
            except Exception:
                robot_state = "manual_driving"
                print("Replaning process failed")
                print("Current robot position is [%i, %i]" % (stop_position[1], stop_position[0]))
                print("Returning to manual driving mode")
                
            
            
            
        elif states[robot_state] == states["load_GB_route"]:
            if GBroute_mode == 2:
                original_pos = route["Init_pos"]
                checkpoint_pos = route["Goal_pos"]
                GBroute_mode = 1
                robot_state = "exe_route"
            elif GBroute_mode == 1:
                # Plan route back
                robot_state = "plan_route"
        
        elif states[robot_state] == states["plan_route"]:
            print("Initiating planning mode...")
            if GBroute_mode == 1:
                # Write reroute file
                config_file = write_config(checkpoint_pos, original_pos)
                
                directory = os.path.dirname(__file__)                
                directory = os.path.join(directory, "planning")
                filename = os.path.join(directory, "main_planning.py")
                
                command = "python3 " + filename + " " + config_file
                # Execute planning
                print("Executing replanning of best path to goal...")
                try:
                    os.system(command)
                except Exception:
                    robot_state = "manual_driving"
                    print("Planning process failed")
                    print("Current robot position is [%i, %i]" % (checkpoint_pos[1], checkpoint_pos[0]))
                    print("Returning to manual driving mode")
                    

                if not os.path.exists(os.path.join(directory, "output", "route.json")):
                    robot_state = "manual_driving"
                    print("Planning process failed")
                    print("Current robot position is [%i, %i]" % (checkpoint_pos[1], checkpoint_pos[0]))
                    print("Returning to manual driving mode")
                    
                else:
                    route_file = os.path.join(directory, "output")
                    route_file = os.path.join(route_file, "route.json") # This file should be created by previous line
                    route = load_path(filename=route_file)
                    GBroute_mode = 0
                    checkpoint_pos = []
                    robot_state = "load_route"
            else:
                directory = os.path.dirname(__file__)                
                directory = os.path.join(directory, "planning")
                filename = os.path.join(directory, "main_planning.py")
                command = "python3 " + filename
                try:
                    os.system(command)
                except Exception:
                    robot_state = "manual_driving"
                    print("Planning process failed")
                if os.path.exists(os.path.join(directory, "output", "route.json")):
                    print("File containing the route can be found on /planning/output/route.json")
                else:
                    print("Planning process failed")
                print("Returning to manual driving mode")
                robot_state = "manual_driving"
        else:
            # Something went wrong if this is executed
            robot_state = "manual_driving"
                
        if no_obs_detected(obs_det_msg, ch):
            key_to_publish = ch
        else:
            print("Obstacle detected while moving")
            obs_det_pos = {0: "front", 1: "left side", 2: "right side", 3: "back"}
            for index, charac in enumerate(obs_det_msg):
                if charac == "o":
                    print("Obstacle position: %s of the robot" % obs_det_pos[index])
#             print(obs_det_msg)
#             print("Robot was trying to send %s" % ch)
            key_to_publish = STOP_KEY
            if states[robot_state] == states["exe_route"]:
                print("Replaning mode started")
                obs_det_flag = True
                robot_state = "replan_route"
            # ~  = path[str(step-1)] # Hay que guardar la posicion en la que se ha quedado el robot
                
        if ser.inWaiting > 0:
            # ~ print("Sending key: %s" % key_to_publish)
            ser.write((key_to_publish+"\n").encode('utf-8'))
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("remote_serialctrl_node")
    remote_control_main()
