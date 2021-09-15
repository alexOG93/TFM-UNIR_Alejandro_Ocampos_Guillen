import map_processing
import path_search
import IO_data_control
import planning_eval


def main_func(input_files):
    eval_data = planning_eval.RunTime()
    # Input files checking
    input_domain = dict()
    for file in input_files:
        # Searchs for a file of parameters configuration
        file_info = IO_data_control.load_json(file)
        if "Init_pos" in file_info.keys():
            input_domain = file_info
    
    # Loading problem domain, if exists
    if input_domain:
        # Parameters loading
        params = IO_data_control.load_params(input_domain["Params_file"])
        IO_data_control.save_params(params)

        # Intial and target robot positions
        init_pos = input_domain["Init_pos"][::-1]
        goal_pos = input_domain["Goal_pos"][::-1]
        
        # Map loading and processing
        map_binary_file = input_domain["Binary_map"]
        map_processed_file = input_domain["Processed_map"]
        map_binary = map_processing.get_bin_map(map_binary_file)
        map_processed = map_processing.get_bin_map(map_processed_file)
        if "Obstacle_pos" in input_domain.keys():
            map_binary, map_processed = map_processing.update_map(map_binary, map_processed, input_domain,
                                                                  params["CAR_SIZE"], params["PIXELS_PER_METER"],
                                                                  params["SAFETY_MARGIN"])
#         map_binary = map_processing.resize_map(map_binary, params)
#         map_processed = map_processing.map_safety_margin(map_binary, params["CAR_SIZE"], params["PIXELS_PER_METER"],
#                                                          params["SAFETY_MARGIN"])

    # Creating problem domain
    else:
        # Parameters loading
        params = IO_data_control.load_params()
        IO_data_control.save_params(params)

        # Map loading and processing
        master = IO_data_control.create_master()
        print("Please, select the image file that contains the map for the route")
        map_file = IO_data_control.choose_map(master)
        print("Processing map file...")
        map_binary = map_processing.get_bin_map(map_file)
#         map_processing.save_map(map_binary, "map_binary_orig.png")
#        map_binary = map_processing.resize_map(map_binary, params)
        IO_data_control.save_params(params)
        map_processed = map_processing.map_safety_margin(map_binary, params["CAR_SIZE"], params["PIXELS_PER_METER"],
                                                         params["SAFETY_MARGIN"])

        # Intial and target robot positions
        print("Please, select input mode for defining robot initial and final positions")
        master.deiconify()
        input_mode = IO_data_control.ModeWindow(master)
        master.mainloop()

        if input_mode.output == "map":
            print("Please, mark the robot initial position, and later the final desired position")
            init_pos = map_processing.get_point(map_processed)
            goal_pos = map_processing.get_point(map_processed)
        elif input_mode.output == "manual":
            print("Please, write robot coordinates for intial and final positions")
            robot_pos = IO_data_control.PositionWindow(master)
            master.mainloop()
            init_pos = robot_pos.init_pos
            goal_pos = robot_pos.goal_pos
        else:
            raise ValueError("Unknown method to input robot positions")

    eval_data.get_map_time()

    map_processing.save_map(map_binary * 255, output_file="map_binary.png")
    map_processing.save_map(map_processed*255, output_file="map_processed.png")

    # Search starting
    print("Search is starting...")
    cost, path, states_num = path_search.find_path(map_processed, init_pos, goal_pos, verbose=True, verbose_file="path_search.log")
    
    print("Search has finished")
    eval_data.get_search_time()
    map_w_path = map_processing.add_path_to_map(map_binary, path, params["CAR_SIZE"], params["PIXELS_PER_METER"])
    
    print("Printing path...")
    steps = IO_data_control.transform_path(path, params["MOV_CODE"], output_file="path_steps.json")
    route = IO_data_control.create_route(steps, params["PIXELS_PER_METER"], output_file="route.json")
    
#     print("Showing map...")
#     
#     map_processing.print_map(map_w_path)
    
    print("Saving map...")
    map_processing.save_map(map_w_path, output_file="map_with_route.png")
    
    print("Execution finished")
    eval_data.save_len(round(cost*100/params["PIXELS_PER_METER"], 2), len(steps)-1, states_num) #Last step is always STOP
    eval_data.write_data()

    return


if __name__ == '__main__':
    arguments = IO_data_control.get_arguments()
    main_func(arguments)
